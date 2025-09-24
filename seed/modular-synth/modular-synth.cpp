#include "daisy_seed.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisysp;

DaisySeed       hw;
MidiUartHandler midi;


constexpr int N_VOICES = 8;

// --- Global waveform states ---
bool sawOn   = true; // botão/LED controlam apenas esta onda
bool pulseOn = true; // botão/LED controlam apenas esta onda

// subVolume global
float subVolume = 0.8f;

// Noise volume
float noiseVolume = 0.0f;

// One voice = 2 Oscillators + Amp Envelope
struct Voice
{
    Oscillator sawOsc;
    Oscillator subSawOsc;
    Oscillator pulseOsc;
    Oscillator subPulseOsc;
    WhiteNoise noise;

    Adsr amp_env;

    bool  gate     = false;
    int   note     = -1;
    float velocity = 0.0f;

    void Init(float samplerate)
    {
        // Saw Oscillator
        sawOsc.Init(samplerate);
        sawOsc.SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
        sawOsc.SetAmp(0.5f);

        // Sub Saw Oscillator
        subSawOsc.Init(samplerate);
        subSawOsc.SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
        subSawOsc.SetAmp(0.25f);

        // Pulse Oscillator
        pulseOsc.Init(samplerate);
        pulseOsc.SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE);
        pulseOsc.SetAmp(0.5f);

        // Sub Pulse Oscillator
        subPulseOsc.Init(samplerate);
        subPulseOsc.SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE);
        subPulseOsc.SetAmp(0.25f);

        // Noise
        noise.Init();

        // Amp envelope
        amp_env.Init(samplerate);
        amp_env.SetAttackTime(0.01f);
        amp_env.SetDecayTime(0.1f);
        amp_env.SetSustainLevel(0.8f);
        amp_env.SetReleaseTime(0.3f);
    }

    void SetFreq(float freq)
    {
        sawOsc.SetFreq(freq);
        subSawOsc.SetFreq(freq / 2.0f);
        pulseOsc.SetFreq(freq);
        subPulseOsc.SetFreq(freq / 2.0f);
    }

    float Process()
    {
        float env = amp_env.Process(gate);

        float sig = 0.0f;
        if(sawOn)
        {
            sig += sawOsc.Process();
            sig += subSawOsc.Process() * subVolume;
        }
        if(pulseOn)
        {
            sig += pulseOsc.Process();
            sig += subPulseOsc.Process() * subVolume;
        }
        sig += noise.Process() * noiseVolume;
        return sig * env * velocity;
    }
};

Voice voices[N_VOICES];
int   voice_index = 0;

// --- Buttons and LEDs ---
// Invertidos: cada botão/LED agora controla a onda oposta
Switch buttonSaw;   // agora controla pulseOn (invertido)
Switch buttonPulse; // agora controla sawOn (invertido)
GPIO   ledSaw;      // agora mostra estado de pulseOn (invertido)
GPIO   ledPulse;    // agora mostra estado de sawOn (invertido)

// --- Audio callback ---
void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    // Lê o knob de subVolume (ADC canal 0) com mapeamento exponencial
    subVolume   = fmap(hw.adc.GetFloat(0), 0.0f, 1.0f, Mapping::EXP);
    noiseVolume = fmap(hw.adc.GetFloat(1), 0.0f, 1.0f, Mapping::EXP) * 0.3f;

    for(size_t i = 0; i < size; i++)
    {
        float mix = 0.0f;

        for(int v = 0; v < N_VOICES; v++)
        {
            mix += voices[v].Process();
        }

        // normalize + aplica subVolume
        mix *= 0.2f;

        out[0][i] = mix;
        out[1][i] = mix;
    }
}

// --- MIDI handler ---
void HandleMidiMessage(MidiEvent m)
{
    switch(m.type)
    {
        case NoteOn:
        {
            NoteOnEvent p = m.AsNoteOn();
            if(p.velocity > 0)
            {
                Voice &v = voices[voice_index];
                v.SetFreq(mtof(p.note));
                v.note = p.note;
                // Aplica mapeamento exponencial na velocity para resposta mais musical
                v.velocity
                    = fmap(p.velocity / 127.0f, 0.0f, 1.0f, Mapping::EXP);
                v.gate = true;
                v.amp_env.Retrigger(false);

                voice_index = (voice_index + 1) % N_VOICES;
            }
            else
            {
                for(auto &v : voices)
                    if(v.note == p.note)
                        v.gate = false;
            }
        }
        break;

        case NoteOff:
        {
            NoteOffEvent p = m.AsNoteOff();
            for(auto &v : voices)
                if(v.note == p.note)
                    v.gate = false;
        }
        break;

        default: break;
    }
}

// --- Main ---
int main(void)
{
    hw.Init();
    hw.SetAudioBlockSize(4);
    hw.usb_handle.Init(UsbHandle::FS_INTERNAL);
    System::Delay(250);

    float samplerate = hw.AudioSampleRate();
    for(auto &v : voices)
        v.Init(samplerate);

    // --- MIDI config ---
    MidiUartHandler::Config midi_cfg;
    midi_cfg.transport_config.periph = UartHandler::Config::Peripheral::USART_1;
    midi.Init(midi_cfg);
    midi.StartReceive();

    // --- Init buttons (with debounce) and LEDs ---
    buttonSaw.Init(seed::D12, 50); // botão que liga/desliga sawOn (invertido)
    buttonPulse.Init(seed::D13,
                     50); // botão que liga/desliga pulseOn (invertido)
    ledSaw.Init(seed::D15, GPIO::Mode::OUTPUT);   // LED saw (invertido)
    ledPulse.Init(seed::D16, GPIO::Mode::OUTPUT); // LED pulse (invertido)


    // Create an array of two AdcChannelConfig objects
    const int        num_adc_channels = 2;
    AdcChannelConfig my_adc_config[num_adc_channels];
    my_adc_config[0].InitSingle(seed::D17);
    my_adc_config[1].InitSingle(seed::D18);
    hw.adc.Init(my_adc_config, num_adc_channels);
    hw.adc.Start();

    // --- Start Audio ---
    hw.StartAudio(AudioCallback);

    while(1)
    {
        // --- MIDI ---
        midi.Listen();
        while(midi.HasEvents())
            HandleMidiMessage(midi.PopEvent());

        // --- Buttons ---
        buttonSaw.Debounce();
        buttonPulse.Debounce();

        if(buttonSaw.RisingEdge())
            pulseOn = !pulseOn; // botão saw agora controla pulse (invertido)

        if(buttonPulse.RisingEdge())
            sawOn = !sawOn; // botão pulse agora controla saw (invertido)

        // LEDs mostram o estado das ondas (invertidos)
        ledSaw.Write(pulseOn); // LED saw mostra estado do pulse
        ledPulse.Write(sawOn); // LED pulse mostra estado do saw

        System::Delay(1);
    }
}
