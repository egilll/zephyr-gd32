# Copyright (c) 2026, Ylhyra ehf.
# SPDX-License-Identifier: Apache-2.0

param(
	[int]$Seconds = 30,
	[switch]$ExerciseVolume
)

$source = @'
using System;
using System.Runtime.InteropServices;
using System.Threading;

public static class Uac1PlaybackTest
{
	[ComImport]
	[Guid("A95664D2-9614-4F35-A746-DE8DB63617E6")]
	[InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
	private interface IMMDeviceEnumerator
	{
		int EnumAudioEndpoints(int dataFlow, uint stateMask, out IntPtr devices);
		int GetDefaultAudioEndpoint(int dataFlow, int role, out IntPtr device);
		int GetDevice([MarshalAs(UnmanagedType.LPWStr)] string id, out IMMDevice device);
	}

	[ComImport]
	[Guid("D666063F-1587-4E43-81F1-B948E807363F")]
	[InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
	private interface IMMDevice
	{
		int Activate(ref Guid iid, uint classContext, IntPtr activationParams,
		             [MarshalAs(UnmanagedType.IUnknown)] out object instance);
	}

	[ComImport]
	[Guid("5CDF2C82-841E-4546-9722-0CF74078229A")]
	[InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
	private interface IAudioEndpointVolume
	{
		int RegisterControlChangeNotify(IntPtr notify);
		int UnregisterControlChangeNotify(IntPtr notify);
		int GetChannelCount(out uint count);
		int SetMasterVolumeLevel(float levelDb, Guid eventContext);
		int SetMasterVolumeLevelScalar(float level, Guid eventContext);
		int GetMasterVolumeLevel(out float levelDb);
		int GetMasterVolumeLevelScalar(out float level);
		int SetChannelVolumeLevel(uint channel, float levelDb, Guid eventContext);
		int SetChannelVolumeLevelScalar(uint channel, float level, Guid eventContext);
		int GetChannelVolumeLevel(uint channel, out float levelDb);
		int GetChannelVolumeLevelScalar(uint channel, out float level);
		int SetMute([MarshalAs(UnmanagedType.Bool)] bool mute, Guid eventContext);
		int GetMute([MarshalAs(UnmanagedType.Bool)] out bool mute);
	}

	[ComImport]
	[Guid("BCDE0395-E52F-467C-8E3D-C4579291692E")]
	private class MMDeviceEnumerator
	{
	}

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Auto)]
    private struct WaveOutCaps
    {
        public ushort ManufacturerId;
        public ushort ProductId;
        public uint DriverVersion;
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 32)] public string ProductName;
        public uint Formats;
        public ushort Channels;
        public ushort Reserved;
        public uint Support;
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct WaveFormat
    {
        public ushort FormatTag;
        public ushort Channels;
        public uint SamplesPerSecond;
        public uint AverageBytesPerSecond;
        public ushort BlockAlign;
        public ushort BitsPerSample;
        public ushort ExtraSize;
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct WaveHeader
    {
        public IntPtr Data;
        public uint BufferLength;
        public uint BytesRecorded;
        public UIntPtr User;
        public uint Flags;
        public uint Loops;
        public IntPtr Next;
        public IntPtr Reserved;
    }

    [DllImport("winmm.dll")]
    private static extern uint waveOutGetNumDevs();

    [DllImport("winmm.dll", CharSet = CharSet.Auto)]
    private static extern int waveOutGetDevCaps(UIntPtr deviceId, out WaveOutCaps caps,
                                                uint capsSize);

    [DllImport("winmm.dll")]
    private static extern int waveOutOpen(out IntPtr handle, uint deviceId,
                                          ref WaveFormat format, IntPtr callback,
                                          IntPtr instance, uint flags);

    [DllImport("winmm.dll")]
    private static extern int waveOutPrepareHeader(IntPtr handle, ref WaveHeader header,
                                                   uint headerSize);

    [DllImport("winmm.dll")]
    private static extern int waveOutWrite(IntPtr handle, ref WaveHeader header,
                                           uint headerSize);

    [DllImport("winmm.dll")]
    private static extern int waveOutUnprepareHeader(IntPtr handle, ref WaveHeader header,
                                                     uint headerSize);

    [DllImport("winmm.dll")]
    private static extern int waveOutClose(IntPtr handle);

    [DllImport("winmm.dll")]
    private static extern int waveOutGetVolume(IntPtr handle, out uint volume);

    [DllImport("winmm.dll")]
    private static extern int waveOutSetVolume(IntPtr handle, uint volume);

    private const uint HeaderDone = 1;

    public static void Run(int seconds, bool exerciseVolume, string endpointId)
    {
        uint deviceId = UInt32.MaxValue;
        WaveOutCaps caps;

        for (uint id = 0; id < waveOutGetNumDevs(); id++) {
            if (waveOutGetDevCaps((UIntPtr)id, out caps,
                                  (uint)Marshal.SizeOf<WaveOutCaps>()) != 0) {
                continue;
            }

            Console.WriteLine("waveOut {0}: {1}", id, caps.ProductName);
            if (caps.ProductName.IndexOf("UAC1", StringComparison.OrdinalIgnoreCase) >= 0) {
                deviceId = id;
            }
        }

        if (deviceId == UInt32.MaxValue) {
            throw new InvalidOperationException("UAC1 waveOut endpoint not found");
        }

        WaveFormat format = new WaveFormat {
            FormatTag = 1,
            Channels = 2,
            SamplesPerSecond = 48000,
            AverageBytesPerSecond = 192000,
            BlockAlign = 4,
            BitsPerSample = 16,
            ExtraSize = 0
        };
        IntPtr handle;
        int result = waveOutOpen(out handle, deviceId, ref format, IntPtr.Zero,
                                 IntPtr.Zero, 0);
        Console.WriteLine("waveOutOpen={0}, device={1}", result, deviceId);
        if (result != 0) {
            return;
        }

        int frameCount = 48000 * seconds;
        byte[] pcm = new byte[frameCount * 4];
        double[] notes = { 261.63, 329.63, 392.00, 523.25, 392.00, 329.63,
                           293.66, 349.23, 440.00, 587.33, 440.00, 349.23 };

        for (int frame = 0; frame < frameCount; frame++) {
            int noteFrame = frame % 24000;
            double frequency = notes[(frame / 24000) % notes.Length];
            double envelope = Math.Min(1.0, noteFrame / 1200.0) *
                              Math.Min(1.0, (24000 - noteFrame) / 1200.0);
            short sample = (short)(Math.Sin(2 * Math.PI * frequency * frame / 48000.0) *
                                   9000 * envelope);
            pcm[frame * 4] = (byte)sample;
            pcm[frame * 4 + 1] = (byte)(sample >> 8);
            pcm[frame * 4 + 2] = (byte)sample;
            pcm[frame * 4 + 3] = (byte)(sample >> 8);
        }

        GCHandle pinned = GCHandle.Alloc(pcm, GCHandleType.Pinned);
        WaveHeader header = new WaveHeader {
            Data = pinned.AddrOfPinnedObject(),
            BufferLength = (uint)pcm.Length
        };
        uint headerSize = (uint)Marshal.SizeOf<WaveHeader>();

        result = waveOutPrepareHeader(handle, ref header, headerSize);
        Console.WriteLine("waveOutPrepareHeader={0}", result);
        if (result == 0) {
            result = waveOutWrite(handle, ref header, headerSize);
        }
        Console.WriteLine("waveOutWrite={0}", result);

        DateTime deadline = DateTime.UtcNow.AddSeconds(seconds + 5);
		DateTime started = DateTime.UtcNow;
		uint originalVolume = UInt32.MaxValue;
		float originalEndpointVolume = 1.0f;
		bool originalMute = false;
		IAudioEndpointVolume endpointVolume = null;
		int volumeStage = 0;
		if (exerciseVolume) {
			Console.WriteLine("waveOutGetVolume={0}",
			                  waveOutGetVolume(handle, out originalVolume));
			IMMDeviceEnumerator enumerator = (IMMDeviceEnumerator)new MMDeviceEnumerator();
			IMMDevice endpoint;
			object volumeObject;
			Guid volumeIid = typeof(IAudioEndpointVolume).GUID;

			if (enumerator.GetDevice(endpointId, out endpoint) != 0 ||
			    endpoint.Activate(ref volumeIid, 23, IntPtr.Zero, out volumeObject) != 0) {
				throw new InvalidOperationException("Could not open UAC1 endpoint volume");
			}
			endpointVolume = (IAudioEndpointVolume)volumeObject;
			endpointVolume.GetMasterVolumeLevelScalar(out originalEndpointVolume);
			endpointVolume.GetMute(out originalMute);
		}
        while ((header.Flags & HeaderDone) == 0 && DateTime.UtcNow < deadline) {
			if (exerciseVolume) {
				double elapsed = (DateTime.UtcNow - started).TotalSeconds;
				if (volumeStage == 0 && elapsed >= 1) {
					Console.WriteLine("endpoint SetVolume(25%)={0}",
					                  endpointVolume.SetMasterVolumeLevelScalar(0.25f, Guid.Empty));
					volumeStage++;
				} else if (volumeStage == 1 && elapsed >= 3) {
					Console.WriteLine("endpoint SetMute={0}",
					                  endpointVolume.SetMute(true, Guid.Empty));
					volumeStage++;
				} else if (volumeStage == 2 && elapsed >= 4) {
					Console.WriteLine("endpoint restore volume={0} mute={1}",
					                  endpointVolume.SetMasterVolumeLevelScalar(
					                      originalEndpointVolume, Guid.Empty),
					                  endpointVolume.SetMute(originalMute, Guid.Empty));
					volumeStage++;
				}
			}
            Thread.Sleep(20);
        }
		if (exerciseVolume && volumeStage < 3) {
			endpointVolume.SetMasterVolumeLevelScalar(originalEndpointVolume, Guid.Empty);
			endpointVolume.SetMute(originalMute, Guid.Empty);
		}

        Console.WriteLine("completed={0}", (header.Flags & HeaderDone) != 0);
        Console.WriteLine("waveOutUnprepareHeader={0}",
                          waveOutUnprepareHeader(handle, ref header, headerSize));
        Console.WriteLine("waveOutClose={0}", waveOutClose(handle));
        pinned.Free();
    }
}
'@

Add-Type -TypeDefinition $source
$endpoint = Get-PnpDevice -Class AudioEndpoint |
	Where-Object FriendlyName -Like '*UAC1*' |
	Select-Object -First 1
if (-not $endpoint) {
	throw 'UAC1 AudioEndpoint not found'
}
$endpointId = $endpoint.InstanceId.Substring('SWD\MMDEVAPI\'.Length)
[Uac1PlaybackTest]::Run($Seconds, $ExerciseVolume.IsPresent, $endpointId)
