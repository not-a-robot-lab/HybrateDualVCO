// PT8211 dual-oscillator (AVR32DB32 / DxCore)
// L en R volledig onafhankelijk: CV + tune (1V/oct), shape (driehoek->zaagtand), gain
// PT8211-data via USART0 in SPI-host-modus (PA4 = DIN, PA6 = BCK, WS op PA5 als GPIO),
// of als fallback bit-banging via VPORTA (USE_HW_SPI 0)
// Smoothing op shape (traag, knop) en gain (licht, zodat envelopes hun attack houden)

#define BCK_bm PIN6_bm
#define DIN_bm PIN4_bm
#define WS_bm  PIN5_bm

#define SAMPLE_RATE 64000UL   // TCB0: CCMP = F_CPU/SAMPLE_RATE - 1 (24 MHz / 64 kHz = 375 cycli per sample)
#define USE_HW_SPI  1         // 1 = USART0 MSPI (snel), 0 = bit-banging (alleen tot ~40 kHz haalbaar)
#define A_MAX       8000      // amplitude bij blokgolf (shape=0), volle gain
#define A_PEAK      32000.0f  // max. DAC-amplitude van de korte fase bij zaagtand
#define F0_BASE     261.63f   // C4
#define SMOOTH_COEF_SHAPE 0.05f   // smoothing op shape-ADC (knop, mag traag)
#define SMOOTH_COEF_GAIN  0.5f    // lichte smoothing op gain-ADC: alleen ruis, envelope-attack blijft snel
// DEBUG: 1 = maak ruwe gain-ADC-waarden hoorbaar (blokgolf, periode in samples = offset + ADC-waarde)
//        L: 200 + rawGainL, R: 1000 + rawGainR. Terugzetten naar 0 voor normaal gebruik!
#define DEBUG_ADC_PROBE 0

#define GAIN_DEADZONE     0.02f   // gain onder 2% (~100 mV) = stilte; envelope rust op ~35-55 mV

// Pin-toewijzing per kanaal (zie schema: shape/gain/tune/CV L+R)
#define ADC_CV_L    PIN_PF3
#define ADC_CV_R    PIN_PF2
#define ADC_TUNE_L  PIN_PF5
#define ADC_TUNE_R  PIN_PD7
#define ADC_SHAPE_L PIN_PD3
#define ADC_SHAPE_R PIN_PD1
#define ADC_GAIN_L  PIN_PF0
#define ADC_GAIN_R  PIN_PF1
#define ADC_FMOD    PIN_PF4   // gedeelde FM-ingang (JA9 via R19-attenuator), zelfde schaling als 1V/oct

#define FM_OCT_PER_VOLT 0.213f  // FM-diepte: -5..+5V op JA9 = -1..+1 octaaf bij R19 volledig open
                                // (0.2 nominaal, +6% voor verlies door 10k parallel over R19; gemeten)

typedef struct {
  // Alleen door de ISR gebruikt (na init): niet volatile, zodat de ISR ze in registers houdt
  int16_t  a_long, a_short;
  uint16_t counter;
  bool     inLongPhase;
  // Gedeeld met loop(): volatile, geschreven met interrupts uit
  volatile uint16_t next_n_long, next_n_short;
  volatile int16_t  next_a_long, next_a_short;
} Oscillator;

Oscillator oscL, oscR;

// Smoothing-state per kanaal
float smoothedGainL = 0.0f, smoothedGainR = 0.0f;
float smoothedShapeL = 0.0f, smoothedShapeR = 0.0f;

// --- Snelle 2^x-benadering (geen powf()/ldexpf() op audio/control-rate) ---
#define POW2_TABLE_SIZE 65
float pow2Table[POW2_TABLE_SIZE];

#define POW2I_RANGE 9
float pow2iTable[2 * POW2I_RANGE + 1];

void buildPow2Table() {
  for (int i = 0; i < POW2_TABLE_SIZE; i++) {
    pow2Table[i] = powf(2.0f, (float)i / (POW2_TABLE_SIZE - 1));
  }
  for (int i = -POW2I_RANGE; i <= POW2I_RANGE; i++) {
    pow2iTable[i + POW2I_RANGE] = powf(2.0f, (float)i);
  }
}

inline float fastPow2(float x) {
  int ix = (int)floorf(x);
  float pos = (x - ix) * (POW2_TABLE_SIZE - 1);
  uint8_t idx = (uint8_t)pos;
  if (idx > POW2_TABLE_SIZE - 2) idx = POW2_TABLE_SIZE - 2;
  float t = pos - idx;
  // Lineaire interpolatie tussen tabelwaarden (zonder: stappen van 18.75 cent)
  float m = pow2Table[idx] + (pow2Table[idx + 1] - pow2Table[idx]) * t;
  if (ix < -POW2I_RANGE) ix = -POW2I_RANGE;
  if (ix >  POW2I_RANGE) ix =  POW2I_RANGE;
  return m * pow2iTable[ix + POW2I_RANGE];
}

#if USE_HW_SPI
// --- PT8211 via USART0 SPI-host: hardware schuift de bits uit (BCK 12 MHz) ---
static inline __attribute__((always_inline)) void spiWrite(uint8_t b) {
  while (!(USART0.STATUS & USART_DREIF_bm));
  USART0.TXDATAL = b;
}

// PT8211 (LSB-justified) neemt de 16 bits vóór elke WS-flank over. In twee helften, zodat
// de ISR R kan uitrekenen terwijl de hardware het L-woord uitschuift (~32 cycli).
// sendLeft: WS laag (neemt vorig R-woord over), start L-woord.
static inline __attribute__((always_inline)) void sendLeft(int16_t left) {
  VPORTA.OUT &= ~WS_bm;
  USART0.STATUS = USART_TXCIF_bm;
  spiWrite((uint16_t)left >> 8); spiWrite((uint8_t)left);
}

// sendRight: wacht tot L volledig uit is (TXCIF), WS hoog (neemt L over), start R-woord.
// Het R-woord wordt overgenomen bij de WS-flank aan het begin van de volgende ISR.
static inline __attribute__((always_inline)) void sendRight(int16_t right) {
  while (!(USART0.STATUS & USART_TXCIF_bm));
  VPORTA.OUT |= WS_bm;
  spiWrite((uint16_t)right >> 8); spiWrite((uint8_t)right);
}

void setupPT8211() {
  VPORTA.DIR |= (BCK_bm | DIN_bm | WS_bm);
  VPORTA.OUT &= ~(BCK_bm | DIN_bm | WS_bm);
  PORTMUX.USARTROUTEA = PORTMUX_USART0_ALT1_gc;   // TxD PA4, XCK PA6 (RxD PA5 niet gebruikt)
  USART0.BAUD  = 1 << 6;                          // MSPI: BCK = F_CPU / 2 = 12 MHz
  USART0.CTRLC = USART_CMODE_MSPI_gc;             // MSB eerst, data geldig op stijgende BCK-flank
  USART0.CTRLB = USART_TXEN_bm;                   // alleen zenden: PA5 blijft gewone GPIO (WS)
}
#else
// --- PT8211 bit-banging (macro-uitgerold: vaste bit-maskers, geen runtime shift) ---
#define SEND_BIT(mask) \
  if (word & (mask)) VPORTA.OUT |= DIN_bm; else VPORTA.OUT &= ~DIN_bm; \
  VPORTA.OUT |= BCK_bm; VPORTA.OUT &= ~BCK_bm;

inline void sendWord(uint16_t word) {
  SEND_BIT(0x8000) SEND_BIT(0x4000) SEND_BIT(0x2000) SEND_BIT(0x1000)
  SEND_BIT(0x0800) SEND_BIT(0x0400) SEND_BIT(0x0200) SEND_BIT(0x0100)
  SEND_BIT(0x0080) SEND_BIT(0x0040) SEND_BIT(0x0020) SEND_BIT(0x0010)
  SEND_BIT(0x0008) SEND_BIT(0x0004) SEND_BIT(0x0002) SEND_BIT(0x0001)
}

inline void sendLeft(int16_t left) {
  VPORTA.OUT &= ~WS_bm;
  sendWord((uint16_t)left);
}

inline void sendRight(int16_t right) {
  VPORTA.OUT |= WS_bm;
  sendWord((uint16_t)right);
}

void setupPT8211() {
  VPORTA.DIR |= (BCK_bm | DIN_bm | WS_bm);
  VPORTA.OUT &= ~(BCK_bm | DIN_bm | WS_bm);
}
#endif

// --- Oscillator-parameterberekening (control-rate, aangeroepen vanuit loop()) ---
// shapeSmoothed/gainSmoothed: al gefilterde 0..1-waarden
void computeOsc(Oscillator *osc, float shapeSmoothed, float gainSmoothed, float freqHz) {
  float totalSamples = SAMPLE_RATE / freqHz;
  if (totalSamples > 65000.0f) totalSamples = 65000.0f;   // CV + tune + FM kan onder ~0.6 Hz komen -> uint16-overflow
  float tLongFrac  = 0.5f + 0.45f * shapeSmoothed;
  float tShortFrac = 1.0f - tLongFrac;

  // Compensatie: bij extreme (zaagtand-achtige) duty-cycles verliest de lekke
  // integrator (bleed-weerstand) veel piek-piekspanning -> hier gecompenseerd.
  float compensation = 0.5f / tShortFrac;

  // aTarget = amplitude van de korte (steile) fase. Mag verder gaan dan A_MAX (tot
  // A_PEAK), zodat de gain bij zaagtand niet al bij ~10% in de clamp loopt.
  // Dode zone: onder GAIN_DEADZONE exact 0, daarboven herschaald zodat 1.0 volle gain blijft (geen sprong).
  float gain = (gainSmoothed - GAIN_DEADZONE) / (1.0f - GAIN_DEADZONE);
  if (gain < 0.0f) gain = 0.0f;

  float aTargetRaw = A_MAX * gain * compensation;
  if (aTargetRaw > A_PEAK) aTargetRaw = A_PEAK;
  if (aTargetRaw < 0.0f)   aTargetRaw = 0.0f;

  // Periode één keer afronden en verdelen, zodat nLong + nShort exact de periode is
  // (twee losse afkappingen verloren tot 2 samples per periode -> te vlak bij hoge tonen).
  uint16_t total     = (uint16_t)(totalSamples + 0.5f);
  uint16_t newNLong  = (uint16_t)(total * tLongFrac + 0.5f);
  if (newNLong < 3) newNLong = 3;     // voorkom extreme afronding bij hoge shape/frequentie
  uint16_t newNShort = (total >= newNLong + 3) ? total - newNLong : 3;

  // DC-vrije puls: a_long * n_long == a_short * n_short. Met gelijke amplitudes had
  // de puls bij shape=1 een DC-component van 0.9*a, die na de lekke integrator als
  // gain-afhankelijke offset meebewoog (envelope hoorbaar als bonk/overshoot).
  int16_t aShort = (int16_t)aTargetRaw;
  int16_t aLong  = (int16_t)(aTargetRaw * newNShort / newNLong);

  // Schrijf naar de "next"-velden; de ISR neemt deze pas over bij de eerstvolgende
  // faseovergang. Interrupts kort uit: 16-bit writes zijn op AVR niet atomair, en
  // de vier velden moeten als set worden overgenomen.
  uint8_t sreg = SREG;
  cli();
  osc->next_n_long = newNLong; osc->next_n_short = newNShort;
  osc->next_a_long = aLong;    osc->next_a_short = aShort;
  SREG = sreg;
}

static inline __attribute__((always_inline)) int16_t stepOsc(Oscillator *osc) {
  int16_t value;
  uint16_t counter = osc->counter;
  if (osc->inLongPhase) {
    // Lopende fase inkorten als de nieuwe periode korter is: anders blijft één
    // foute (zeer lage) pitch-lezing tot ~250 ms "hangen" in één lange fase.
    uint16_t n = osc->next_n_long;
    if (counter > n) counter = n;
    value = osc->a_long;
    if (--counter == 0) {
      osc->a_long  = osc->next_a_long;
      osc->a_short = osc->next_a_short;
      counter = osc->next_n_short;
      osc->inLongPhase = false;
    }
  } else {
    uint16_t n = osc->next_n_short;
    if (counter > n) counter = n;
    value = -osc->a_short;
    if (--counter == 0) {
      osc->a_long  = osc->next_a_long;
      osc->a_short = osc->next_a_short;
      counter = osc->next_n_long;
      osc->inLongPhase = true;
    }
  }
  osc->counter = counter;
  return value;
}

ISR(TCB0_INT_vect) {
  TCB0.INTFLAGS = TCB_CAPT_bm;
  sendLeft(stepOsc(&oscL));
  sendRight(stepOsc(&oscR));   // R wordt berekend terwijl L nog wordt uitgeschoven
}

// --- ADC: directe registeraansturing (DxCore's analogRead() bleek onbetrouwbaar
// op deze chip/core-versie -- bleef rond de helft van het bereik hangen) ---
// Twee conversies, alleen de tweede telt: de sample-condensator houdt nog lading van
// het vorige kanaal vast, en via de 100k-ingangen (gain/shape/FM) laadt die niet
// volledig om (gemeten: gain L las 256 i.p.v. ~38 na de tune-pot).
uint16_t readAdc(uint8_t pin) {
  ADC0.MUXPOS = digitalPinToAnalogInput(pin);
  for (uint8_t i = 0; i < 2; i++) {
    ADC0.COMMAND = ADC_STCONV_bm;
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
    ADC0.INTFLAGS = ADC_RESRDY_bm;
  }
  return ADC0.RES;
}

// --- OP0: PD2 = VDD - V(PD3) (geïnverteerde shape L naar de schakelaar U5) ---
// De omkering gebeurt in firmware: ADC leest shape L, DAC0 krijgt VDD - shape, OP0 buffert
// de DAC als spanningsvolger naar PD2. Eerder zat de interne weerstandsladder aan PD3; die
// vormde met R10 (100k) een deler naar VDD/2, waardoor shape L bij 100% niet tot 5V kwam.
void setupOpamp() {
  PORTD.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;   // analoge pinnen: digitale input-buffer uit
  PORTD.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTD.PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc;   // DAC-uitgang (niet aangesloten)

  VREF.DAC0REF = VREF_REFSEL_VDD_gc;
  DAC0.DATA    = 0;
  DAC0.CTRLA   = DAC_ENABLE_bm | DAC_OUTEN_bm;

  OPAMP.TIMEBASE  = ((F_CPU + 999999UL) / 1000000UL) - 1;
  OPAMP.PWRCTRL   = OPAMP_PWRCTRL_IRSEL_FULL_gc;   // rail-to-rail ingang
  OPAMP.OP0RESMUX = 0;                             // ladder uit: PD3 niet meer belast
  OPAMP.OP0INMUX  = OPAMP_OP0INMUX_MUXPOS_DAC_gc | OPAMP_OP0INMUX_MUXNEG_OUT_gc;  // volger
  OPAMP.OP0CTRLA  = OPAMP_OP0CTRLA_OUTMODE_NORMAL_gc | OPAMP_ALWAYSON_bm;
  OPAMP.CTRLA     = OPAMP_ENABLE_bm;
}

// 12-bit ADC-waarde -> 10-bit DAC (links uitgelijnd in DATA[15:6]), omgekeerd
inline void writeShapeLInv(uint16_t rawShape) {
  DAC0.DATA = (uint16_t)(1023 - (rawShape >> 2)) << 6;
}

void initOscillator(Oscillator *osc) {
  computeOsc(osc, 0.0f, 1.0f, F0_BASE);
  osc->a_long = osc->next_a_long; osc->a_short = osc->next_a_short;
  osc->counter = osc->next_n_long; osc->inLongPhase = true;
}

void setup() {
  setupPT8211();

  buildPow2Table();

  VREF.ADC0REF = VREF_REFSEL_VDD_gc;
  ADC0.CTRLC   = ADC_PRESC_DIV16_gc;
  ADC0.CTRLA   = ADC_RESSEL_12BIT_gc | ADC_ENABLE_bm;
  ADC0.SAMPCTRL = 14;   // sample-tijd 16 ADC-klokken (~11 us): shape/gain/FM komen via 100k binnen

  setupOpamp();

  initOscillator(&oscL);
  initOscillator(&oscR);

  // TCB0: audio-sample-timer. CCMP = (F_CPU/SAMPLE_RATE) - 1.
  // BELANGRIJK: de ISR moet ruim binnen F_CPU/SAMPLE_RATE cycli blijven, anders komt de
  // werkelijke sample-rate NIET overeen met SAMPLE_RATE en klinkt alles te laag
  // (projectgeschiedenis: bit-banging met variabele bit-shift gaf 440Hz -> ~140Hz).
  // Met USE_HW_SPI: ~155 cycli typisch, ~215 als beide oscillatoren tegelijk van fase
  // wisselen. Bit-banging (USE_HW_SPI 0) kost ~400 cycli: dan SAMPLE_RATE terug naar 40000.
  TCB0.CCMP = (F_CPU / SAMPLE_RATE) - 1;
  TCB0.CTRLB = TCB_CNTMODE_INT_gc;
  TCB0.INTCTRL = TCB_CAPT_bm;
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
}

void loop() {
  // --- FM (gedeeld door L en R): 0..5V ADC -> -5..+5V op de jack ---
  uint16_t rawFm = readAdc(ADC_FMOD);
  float octFm = (2.0f * ((float)rawFm / 4095.0f * 5.0f) - 5.0f) * FM_OCT_PER_VOLT;

  // --- Linkerkanaal ---
  uint16_t rawShapeL = readAdc(ADC_SHAPE_L);
  writeShapeLInv(rawShapeL);
  uint16_t rawCVL    = readAdc(ADC_CV_L);
  uint16_t rawTuneL  = readAdc(ADC_TUNE_L);
  uint16_t rawGainL  = readAdc(ADC_GAIN_L);

  smoothedShapeL += ((float)rawShapeL / 4095.0f - smoothedShapeL) * SMOOTH_COEF_SHAPE;
  smoothedGainL  += ((float)rawGainL  / 4095.0f - smoothedGainL)  * SMOOTH_COEF_GAIN;

  float vAdcCVL  = (float)rawCVL / 4095.0f * 5.0f;
  float octCVL   = 2.0f * vAdcCVL - 5.0f;              // 10 octaven totaal, gecentreerd op 0V
  float octTuneL = ((float)rawTuneL / 4095.0f - 0.5f) * 2.0f;  // +-1 octaaf
  float freqL = F0_BASE * fastPow2(octCVL + octTuneL + octFm);

#if DEBUG_ADC_PROBE
  probeOsc(&oscL, 200 + rawGainL);
#else
  computeOsc(&oscL, smoothedShapeL, smoothedGainL, freqL);
#endif

  // --- Rechterkanaal ---
  uint16_t rawShapeR = readAdc(ADC_SHAPE_R);
  uint16_t rawCVR    = readAdc(ADC_CV_R);
  uint16_t rawTuneR  = readAdc(ADC_TUNE_R);
  uint16_t rawGainR  = readAdc(ADC_GAIN_R);

  smoothedShapeR += ((float)rawShapeR / 4095.0f - smoothedShapeR) * SMOOTH_COEF_SHAPE;
  smoothedGainR  += ((float)rawGainR  / 4095.0f - smoothedGainR)  * SMOOTH_COEF_GAIN;

  float vAdcCVR  = (float)rawCVR / 4095.0f * 5.0f;
  float octCVR   = 2.0f * vAdcCVR - 5.0f;
  float octTuneR = ((float)rawTuneR / 4095.0f - 0.5f) * 2.0f;
  float freqR = F0_BASE * fastPow2(octCVR + octTuneR + octFm);

#if DEBUG_ADC_PROBE
  probeOsc(&oscR, 1000 + rawGainR);
#else
  computeOsc(&oscR, smoothedShapeR, smoothedGainR, freqR);
#endif
}