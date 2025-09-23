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

// One voice = 2 Oscillators + Amp Envelope
struct Voice
{
    Oscillator sawOsc;
    Oscillator pulseOsc;
    Adsr       amp_env;

    bool  gate     = false;
    int   note     = -1;
    float velocity = 0.0f;

    void Init(float samplerate)
    {
        // Saw Oscillator
        sawOsc.Init(samplerate);
        sawOsc.SetWaveform(Oscillator::WAVE_POLYBLEP_SAW);
        sawOsc.SetAmp(0.5f);

        // Pulse/Square Oscillator
        pulseOsc.Init(samplerate);
        pulseOsc.SetWaveform(Oscillator::WAVE_POLYBLEP_SQUARE);
        pulseOsc.SetAmp(0.5f);

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
        pulseOsc.SetFreq(freq);
    }

    float Process()
    {
        float env = amp_env.Process(gate);

        float sig = 0.0f;
        if(sawOn)
            sig += sawOsc.Process();
        if(pulseOn)
            sig += pulseOsc.Process();

        return sig * env * velocity;
    }
};

Voice voices[N_VOICES];
int   voice_index = 0;

// --- Buttons and LEDs ---
// Pode trocar os pinos se quiser, mas cada botão corresponde à sua onda
Switch buttonSaw;   // controla sawOn
Switch buttonPulse; // controla pulseOn
GPIO   ledSaw;      // mostra estado de sawOn
GPIO   ledPulse;    // mostra estado de pulseOn

// --- Audio callback ---
void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    for(size_t i = 0; i < size; i++)
    {
        float mix = 0.0f;

        for(int v = 0; v < N_VOICES; v++)
        {
            mix += voices[v].Process();
        }

        // normalize
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
                v.note     = p.note;
                v.velocity = p.velocity / 127.0f;
                v.gate     = true;
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

    // MIDI config
    MidiUartHandler::Config midi_cfg;
    midi_cfg.transport_config.periph = UartHandler::Config::Peripheral::USART_1;
    midi.Init(midi_cfg);
    midi.StartReceive();

    // Init buttons (with debounce) and LEDs
    buttonSaw.Init(seed::D12, 50);   // botão que liga/desliga sawOn
    buttonPulse.Init(seed::D13, 50); // botão que liga/desliga pulseOn
    ledSaw.Init(seed::D15, GPIO::Mode::OUTPUT);
    ledPulse.Init(seed::D16, GPIO::Mode::OUTPUT);

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
            sawOn = !sawOn;

        if(buttonPulse.RisingEdge())
            pulseOn = !pulseOn;

        // LEDs mostram exatamente o estado de cada onda
        ledSaw.Write(sawOn);
        ledPulse.Write(pulseOn);

        System::Delay(1);
    }
}
