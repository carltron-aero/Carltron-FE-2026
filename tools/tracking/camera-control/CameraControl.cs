using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Runtime.InteropServices.ComTypes;
using System.Threading;

namespace Carltron.CameraControl {
    [ComImport, Guid("29840822-5B84-11D0-BD3B-00A0C911CE86"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
    interface ICreateDevEnum { [PreserveSig] int CreateClassEnumerator(ref Guid category, out IEnumMoniker items, int flags); }
    [ComImport, Guid("55272A00-42CB-11CE-8135-00AA004BB851"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
    interface IPropertyBag {
        [PreserveSig] int Read([MarshalAs(UnmanagedType.LPWStr)] string name, [In, Out, MarshalAs(UnmanagedType.Struct)] ref object value, IntPtr errorLog);
        [PreserveSig] int Write([MarshalAs(UnmanagedType.LPWStr)] string name, [In, MarshalAs(UnmanagedType.Struct)] ref object value);
    }
    [ComImport, Guid("C6E13370-30AC-11D0-A18C-00A0C9118956"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
    interface IAMCameraControl {
        [PreserveSig] int GetRange(int property, out int min, out int max, out int step, out int def, out int caps);
        [PreserveSig] int Set(int property, int value, int flags);
        [PreserveSig] int Get(int property, out int value, out int flags);
    }
    [ComImport, Guid("C6E13360-30AC-11D0-A18C-00A0C9118956"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
    interface IAMVideoProcAmp {
        [PreserveSig] int GetRange(int property, out int min, out int max, out int step, out int def, out int caps);
        [PreserveSig] int Set(int property, int value, int flags);
        [PreserveSig] int Get(int property, out int value, out int flags);
    }
    [ComImport, Guid("28F54685-06FD-11D2-B27A-00A0C9223196"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
    interface IKsControl {
        [PreserveSig] int KsProperty(IntPtr property, int propertyLength, IntPtr data, int dataLength, out int returned);
        [PreserveSig] int KsMethod(IntPtr method, int methodLength, IntPtr data, int dataLength, out int returned);
        [PreserveSig] int KsEvent(IntPtr evt, int eventLength, IntPtr data, int dataLength, out int returned);
    }
    [ComImport, Guid("720D4AC0-7533-11D0-A5D6-28DB04C10000"), InterfaceType(ComInterfaceType.InterfaceIsIUnknown)]
    interface ITopology {
        [PreserveSig] int NumCategories(out int count);
        [PreserveSig] int Category(int index, out Guid guid);
        [PreserveSig] int NumConnections(out int count);
        [PreserveSig] int Connection(int index, IntPtr info);
        [PreserveSig] int NodeName(int id, IntPtr name, int size, out int length);
        [PreserveSig] int NumNodes(out int count);
        [PreserveSig] int NodeType(int id, out Guid guid);
        [PreserveSig] int CreateNode(int id, ref Guid iid, [MarshalAs(UnmanagedType.IUnknown)] out object instance);
    }
    public sealed class DeviceInfo { public int Index; public string Name; public string Path; }
    public sealed class ControlInfo {
        public string Group; public string Name; public int Id; public bool Supported;
        public int? Min, Max, Step, Default, Value, Flags, Caps;
        public bool Manual, Auto; public string Mode, Error;
        public double? ExposureSeconds;
    }
    public sealed class KsInfo {
        public string Name; public string Group = "camera"; public int Id; public bool Readable, Writable;
        public int? Value, Flags, Caps, Min, Max, Step;
        public string SupportHresult, GetHresult;
    }
    public sealed class AbsoluteInfo {
        public bool Readable, Writable;
        public int? Pan, Tilt, PanMin, PanMax, PanStep, TiltMin, TiltMax, TiltStep;
        public int Node = -1;
        public string Units = "arcseconds", Error;
    }
    public sealed class VendorInfo {
        public string Name, Group, DataHex, Error;
        public int Selector, Node;
        public bool Readable, AdvertisesWrite;
        public bool AppWritable = false;
    }
    public sealed class State {
        public DeviceInfo Device;
        public List<ControlInfo> Controls = new List<ControlInfo>();
        public List<KsInfo> Extended = new List<KsInfo>();
        public bool PanJog, TiltJog;
        public AbsoluteInfo Absolute;
        public List<VendorInfo> Vendor = new List<VendorInfo>();
    }
    public sealed class Session : IDisposable {
        object filter;
        IAMCameraControl cc;
        IAMVideoProcAmp vp;
        IKsControl ks;
        DeviceInfo device;
        bool moving;
        int motorNode = -1, videoNode = -1, deviceNode = -1;
        static readonly Guid MotorSet = new Guid("ffe52d21-8030-4e2c-82d9-f587d00540bd");
        static readonly Guid VendorVideoSet = new Guid("49e40215-f434-47fe-b158-0e885023e51b");
        static readonly Guid DeviceSet = new Guid("69678ee4-410f-40db-a850-7420d7d8240e");
        static readonly Guid CameraSet = new Guid("C6E13370-30AC-11D0-A18C-00A0C9118956");
        static readonly Guid VideoSet = new Guid("C6E13360-30AC-11D0-A18C-00A0C9118956");
        static readonly string[] CameraNames = { "Pan", "Tilt", "Roll", "Zoom", "Exposure", "Iris", "Focus" };
        static readonly string[] VideoNames = { "Brightness", "Contrast", "Hue", "Saturation", "Sharpness", "Gamma", "ColorEnable", "WhiteBalance", "BacklightCompensation", "Gain", "DigitalMultiplier", "DigitalMultiplierLimit", "WhiteBalanceComponent", "PowerLineFrequency" };
        static readonly string[] ExtendedNames = { "Pan", "Tilt", "Roll", "Zoom", "Exposure", "Iris", "Focus", "ScanMode", "Privacy", "PanTilt", "PanRelative", "TiltRelative", "RollRelative", "ZoomRelative", "ExposureRelative", "IrisRelative", "FocusRelative", "PanTiltRelative", "FocalLength", "AutoExposurePriority" };
        static void Release(object value) { if (value != null && Marshal.IsComObject(value)) Marshal.ReleaseComObject(value); }
        static string Hex(int value) { return "0x" + ((uint)value).ToString("X8"); }
        static void Check(int hr) { if (hr < 0) Marshal.ThrowExceptionForHR(hr); }

        static List<DeviceInfo> Enumerate(int bindIndex, out object selected) {
            selected = null;
            var result = new List<DeviceInfo>();
            object system = null;
            IEnumMoniker items = null;
            try {
                system = Activator.CreateInstance(Type.GetTypeFromCLSID(new Guid("62BE5D10-60EB-11D0-BD3B-00A0C911CE86")));
                var category = new Guid("860BB310-5D01-11D0-BD3B-00A0C911CE86");
                int hr = ((ICreateDevEnum)system).CreateClassEnumerator(ref category, out items, 0);
                if (hr == 1) return result;
                Check(hr);
                var next = new IMoniker[1];
                while (items.Next(1, next, IntPtr.Zero) == 0) {
                    object bag = null;
                    try {
                        var id = new Guid("55272A00-42CB-11CE-8135-00AA004BB851");
                        next[0].BindToStorage(null, null, ref id, out bag);
                        object name = null;
                        Check(((IPropertyBag)bag).Read("FriendlyName", ref name, IntPtr.Zero));
                        string friendly = Convert.ToString(name);
                        if (friendly.IndexOf("PTZ Pro 2", StringComparison.OrdinalIgnoreCase) < 0) continue;
                        string path;
                        next[0].GetDisplayName(null, null, out path);
                        var info = new DeviceInfo { Index = result.Count, Name = friendly, Path = path };
                        result.Add(info);
                        if (info.Index == bindIndex) {
                            var unknown = new Guid("00000000-0000-0000-C000-000000000046");
                            next[0].BindToObject(null, null, ref unknown, out selected);
                        }
                    } finally { Release(bag); Release(next[0]); }
                }
                return result;
            } finally { Release(items); Release(system); }
        }
        public static DeviceInfo[] Devices() { object unused; return Enumerate(-1, out unused).ToArray(); }
        public Session(int index) {
            var list = Enumerate(index, out filter);
            if (index < 0 || index >= list.Count || filter == null) throw new ArgumentException("The selected PTZ Pro 2 is not connected.");
            device = list[index]; cc = filter as IAMCameraControl; vp = filter as IAMVideoProcAmp; ks = filter as IKsControl;
            motorNode = FindNode(MotorSet, 12);
            videoNode = FindNode(VendorVideoSet, 2);
            deviceNode = FindNode(DeviceSet, 1);
        }
        // KSP_NODE is a 32-byte topology request. Only known properties are used.
        byte[] NodeProperty(Guid set, int selector, int node, int flags, byte[] data, out int hr) {
            var header = new byte[32];
            Array.Copy(set.ToByteArray(), header, 16);
            Array.Copy(BitConverter.GetBytes(selector), 0, header, 16, 4);
            Array.Copy(BitConverter.GetBytes(0x10000000 | flags), 0, header, 20, 4);
            Array.Copy(BitConverter.GetBytes(node), 0, header, 24, 4);
            IntPtr prop = Marshal.AllocHGlobal(32), buffer = Marshal.AllocHGlobal(data.Length);
            try {
                Marshal.Copy(header, 0, prop, 32); Marshal.Copy(data, 0, buffer, data.Length);
                int returned = 0;
                hr = ks == null ? unchecked((int)0x80004002) : ks.KsProperty(prop, 32, buffer, data.Length, out returned);
                // SET may return no bytes. GET/support must not parse unreturned data.
                int length = flags == 2 ? data.Length : Math.Min(data.Length, Math.Max(0, returned));
                var result = new byte[length];
                if (length > 0) Marshal.Copy(buffer, result, 0, length);
                return result;
            } finally { Marshal.FreeHGlobal(buffer); Marshal.FreeHGlobal(prop); }
        }
        int FindNode(Guid set, int selector) {
            var topology = filter as ITopology;
            int count;
            if (topology == null || topology.NumNodes(out count) < 0 || count < 0 || count > 128) return -1;
            for (int node = 0; node < count; node++) {
                Guid type;
                if (topology.NodeType(node, out type) < 0 || type != new Guid("941c7ac0-c559-11d0-8a2b-00a0c9255ac1")) continue;
                int hr;
                var support = NodeProperty(set, selector, node, 0x200, new byte[4], out hr);
                if (hr >= 0 && support.Length == 4 && (BitConverter.ToInt32(support, 0) & 1) != 0) return node;
            }
            return -1;
        }
        public AbsoluteInfo ReadAbsolute() {
            var r = new AbsoluteInfo { Node = motorNode };
            if (motorNode < 0) { r.Error = "Absolute interface unavailable."; return r; }
            int hr;
            var support = NodeProperty(MotorSet, 12, motorNode, 0x200, new byte[512], out hr);
            if (hr < 0 || support.Length < 4) { r.Error = "Support query " + Hex(hr); return r; }
            int access = BitConverter.ToInt32(support, 0);
            if ((access & 1) != 0) {
                var value = NodeProperty(MotorSet, 12, motorNode, 1, new byte[8], out hr);
                if (hr >= 0 && value.Length == 8) { r.Readable = true; r.Pan = BitConverter.ToInt32(value, 0); r.Tilt = BitConverter.ToInt32(value, 4); }
                else r.Error = "Position read " + Hex(hr);
            }
            // Recognise the measured two-axis stepped-range description; fail closed otherwise.
            if (support.Length >= 80 && BitConverter.ToInt32(support, 4) == 80
                && BitConverter.ToInt32(support, 32) == 1 && BitConverter.ToInt32(support, 40) == 1
                && BitConverter.ToInt32(support, 44) == 24 && BitConverter.ToInt32(support, 48) == 1) {
                r.PanStep = BitConverter.ToInt32(support, 56); r.TiltStep = BitConverter.ToInt32(support, 60);
                r.PanMin = BitConverter.ToInt32(support, 64); r.TiltMin = BitConverter.ToInt32(support, 68);
                r.PanMax = BitConverter.ToInt32(support, 72); r.TiltMax = BitConverter.ToInt32(support, 76);
                r.Writable = r.Readable && (access & 2) != 0 && r.PanStep > 0 && r.TiltStep > 0
                    && r.PanMin < r.PanMax && r.TiltMin < r.TiltMax
                    && r.PanMin >= -648000 && r.PanMax <= 648000 && r.TiltMin >= -324000 && r.TiltMax <= 324000;
            }
            if (!r.Writable && r.Error == null) r.Error = "No verified writable range; position is read-only.";
            return r;
        }
        public static byte[] AbsolutePayload(AbsoluteInfo c, int pan, int tilt) {
            if (c == null || !c.Writable || !c.PanMin.HasValue || !c.PanMax.HasValue || !c.PanStep.HasValue
                || !c.TiltMin.HasValue || !c.TiltMax.HasValue || !c.TiltStep.HasValue || c.PanStep <= 0 || c.TiltStep <= 0)
                throw new InvalidOperationException("Absolute movement is unsupported or lacks a verified range.");
            if (pan < c.PanMin || pan > c.PanMax || ((long)pan - c.PanMin.Value) % c.PanStep.Value != 0
                || tilt < c.TiltMin || tilt > c.TiltMax || ((long)tilt - c.TiltMin.Value) % c.TiltStep.Value != 0)
                throw new ArgumentOutOfRangeException("position", "Target must match the camera's supported range and step.");
            var data = new byte[8];
            Array.Copy(BitConverter.GetBytes(pan), 0, data, 0, 4);
            Array.Copy(BitConverter.GetBytes(tilt), 0, data, 4, 4);
            return data;
        }
        public AbsoluteInfo SetAbsolute(int pan, int tilt) {
            var data = AbsolutePayload(ReadAbsolute(), pan, tilt);
            int hr;
            NodeProperty(MotorSet, 12, motorNode, 2, data, out hr); Check(hr);
            moving = true;
            // This is logical read-back, not an arrival or settling confirmation.
            return ReadAbsolute();
        }
        VendorInfo ReadVendor(string name, string group, Guid set, int node, int selector, int size) {
            var r = new VendorInfo { Name = name, Group = group, Node = node, Selector = selector };
            if (node < 0) { r.Error = "Extension unavailable."; return r; }
            int hr;
            var support = NodeProperty(set, selector, node, 0x200, new byte[4], out hr);
            if (hr < 0 || support.Length != 4) { r.Error = "Support query " + Hex(hr); return r; }
            int access = BitConverter.ToInt32(support, 0); r.AdvertisesWrite = (access & 2) != 0;
            if ((access & 1) == 0) return r;
            var value = NodeProperty(set, selector, node, 1, new byte[size], out hr);
            r.Readable = hr >= 0 && value.Length == size;
            if (r.Readable) r.DataHex = BitConverter.ToString(value).Replace("-", "");
            else r.Error = "Read query " + Hex(hr);
            return r;
        }
        ControlInfo ReadControl(string group, int p) {
            bool camera = group == "camera";
            var names = camera ? CameraNames : VideoNames;
            if (p < 0 || p >= names.Length) throw new ArgumentException("Unknown control.");
            var r = new ControlInfo { Group = group, Id = p, Name = names[p] };
            int min = 0, max = 0, step = 0, def = 0, caps = 0, value = 0, flags = 0;
            int range = unchecked((int)0x80004002), get = range;
            if (camera && cc != null) { range = cc.GetRange(p, out min, out max, out step, out def, out caps); get = cc.Get(p, out value, out flags); }
            if (!camera && vp != null) { range = vp.GetRange(p, out min, out max, out step, out def, out caps); get = vp.Get(p, out value, out flags); }
            r.Supported = range >= 0 && get >= 0;
            if (range >= 0) { r.Min = min; r.Max = max; r.Step = Math.Max(1, step); r.Default = def; r.Caps = caps; r.Manual = (caps & 2) != 0; r.Auto = (caps & 1) != 0; }
            // Power-line frequency is an unflagged scalar, unlike photographic auto/manual controls.
            if (!camera && p == 13 && range >= 0) r.Manual = true;
            if (get >= 0) { r.Value = value; r.Flags = flags; r.Mode = (flags & 1) != 0 ? "auto" : (flags & 2) != 0 ? "manual" : "unknown"; if (camera && p == 4) r.ExposureSeconds = Math.Pow(2, value); }
            if (!r.Supported) r.Error = "Range " + Hex(range) + "; read " + Hex(get);
            return r;
        }
        // KS property requests use the documented KSPROPERTY_CAMERACONTROL_S layout.
        byte[] Ks(Guid set, int id, int requestFlags, byte[] data, out int hr) {
            var header = new byte[24];
            Array.Copy(set.ToByteArray(), header, 16);
            Array.Copy(BitConverter.GetBytes(id), 0, header, 16, 4);
            Array.Copy(BitConverter.GetBytes(requestFlags), 0, header, 20, 4);
            var propPtr = Marshal.AllocHGlobal(header.Length);
            var dataPtr = Marshal.AllocHGlobal(data.Length);
            try {
                Marshal.Copy(header, 0, propPtr, header.Length);
                Marshal.Copy(data, 0, dataPtr, data.Length);
                int returned;
                hr = ks == null ? unchecked((int)0x80004002) : ks.KsProperty(propPtr, header.Length, dataPtr, data.Length, out returned);
                Marshal.Copy(dataPtr, data, 0, data.Length);
                return data;
            } finally { Marshal.FreeHGlobal(dataPtr); Marshal.FreeHGlobal(propPtr); }
        }
        KsInfo ReadKs(int p) {
            var r = new KsInfo { Id = p, Name = ExtendedNames[p] };
            int hr;
            var support = Ks(CameraSet, p, 0x200, new byte[4], out hr); // BASIC SUPPORT
            r.SupportHresult = Hex(hr);
            if (hr >= 0) { int flags = BitConverter.ToInt32(support, 0); r.Readable = (flags & 1) != 0; r.Writable = (flags & 2) != 0; }
            // Returned camera payload includes the property header plus Value/Flags/Capabilities.
            var read = Ks(CameraSet, p, 1, new byte[40], out hr);
            r.GetHresult = Hex(hr);
            if (hr >= 0) { r.Readable = true; r.Value = BitConverter.ToInt32(read, 24); r.Flags = BitConverter.ToInt32(read, 28); r.Caps = BitConverter.ToInt32(read, 32); }
            if (r.Writable) {
                int lo, hi, delta, def, caps;
                if (cc != null && cc.GetRange(p, out lo, out hi, out delta, out def, out caps) >= 0) { r.Min = lo; r.Max = hi; r.Step = delta; }
                var description = Ks(CameraSet, p, 0x200, new byte[1024], out hr);
                if (hr >= 0 && BitConverter.ToInt32(description, 4) >= 56 && BitConverter.ToInt32(description, 32) > 0) {
                    int kind = BitConverter.ToInt32(description, 40), size = BitConverter.ToInt32(description, 44);
                    if (kind == 2 && size == 16) { r.Step = BitConverter.ToInt32(description, 56); r.Min = BitConverter.ToInt32(description, 64); r.Max = BitConverter.ToInt32(description, 68); }
                    if (kind == 1 && size == 8) { r.Step = 1; r.Min = BitConverter.ToInt32(description, 56); r.Max = BitConverter.ToInt32(description, 60); }
                }
            }
            return r;
        }
        KsInfo ReadPowerLine() {
            var r = new KsInfo { Group = "video", Id = 13, Name = "PowerLineFrequency" };
            var direct = ReadControl("video", 13);
            if (direct.Supported) { r.Readable = true; r.Writable = true; r.Value = direct.Value; r.Min = direct.Min; r.Max = direct.Max; r.Step = direct.Step; r.GetHresult = "0x00000000"; return r; }
            int hr;
            var support = Ks(VideoSet, 13, 0x200, new byte[4], out hr);
            r.SupportHresult = Hex(hr);
            if (hr >= 0) { int flags = BitConverter.ToInt32(support, 0); r.Readable = (flags & 1) != 0; r.Writable = (flags & 2) != 0; }
            var data = Ks(VideoSet, 13, 1, new byte[128], out hr);
            r.GetHresult = Hex(hr);
            if (hr >= 0) { r.Value = BitConverter.ToInt32(data, 24); r.Flags = BitConverter.ToInt32(data, 28); r.Caps = BitConverter.ToInt32(data, 32); }
            return r;
        }
        void WriteKs(int p, int value, int mode) {
            int hr;
            var data = new byte[40];
            Array.Copy(CameraSet.ToByteArray(), data, 16);
            Array.Copy(BitConverter.GetBytes(p), 0, data, 16, 4);
            Array.Copy(BitConverter.GetBytes(2), 0, data, 20, 4);
            Array.Copy(BitConverter.GetBytes(value), 0, data, 24, 4);
            Array.Copy(BitConverter.GetBytes(mode), 0, data, 28, 4);
            Ks(CameraSet, p, 2, data, out hr); Check(hr);
        }
        public State Read() {
            var state = new State { Device = device };
            for (int p = 0; p < CameraNames.Length; p++) state.Controls.Add(ReadControl("camera", p));
            for (int p = 0; p < VideoNames.Length; p++) state.Controls.Add(ReadControl("video", p));
            for (int p = 7; p < ExtendedNames.Length; p++) state.Extended.Add(ReadKs(p));
            state.Extended.Add(ReadPowerLine());
            state.PanJog = state.Extended.Find(p => p.Id == 10).Writable;
            state.TiltJog = state.Extended.Find(p => p.Id == 11).Writable;
            state.Absolute = ReadAbsolute();
            state.Vendor.Add(ReadVendor("Firmware bytes", "device", DeviceSet, deviceNode, 1, 4));
            state.Vendor.Add(ReadVendor("Sensor code", "device", DeviceSet, deviceNode, 4, 2));
            state.Vendor.Add(ReadVendor("Processor code", "device", DeviceSet, deviceNode, 5, 2));
            state.Vendor.Add(ReadVendor("Native mode forced", "video", VendorVideoSet, videoNode, 2, 1));
            state.Vendor.Add(ReadVendor("RightLight", "video", VendorVideoSet, videoNode, 4, 1));
            state.Vendor.Add(ReadVendor("Sensor crop", "video", VendorVideoSet, videoNode, 8, 6));
            state.Vendor.Add(ReadVendor("MJPEG compression", "video", VendorVideoSet, videoNode, 11, 2));
            return state;
        }
        public ControlInfo Set(string group, int id, int value, bool auto) {
            if (group != "camera" && group != "video") throw new ArgumentException("Unknown control group.");
            var c = ReadControl(group, id);
            if (!c.Supported) throw new InvalidOperationException("Control is unsupported.");
            if (auto && !c.Auto || !auto && !c.Manual) throw new ArgumentException("Requested mode is unsupported.");
            if (value < c.Min || value > c.Max || (value - c.Min) % c.Step != 0) throw new ArgumentOutOfRangeException("value", "Value is outside the supported range/step.");
            Check(group == "camera" ? cc.Set(id, value, auto ? 1 : 2) : vp.Set(id, value, id == 13 ? 0 : auto ? 1 : 2));
            return ReadControl(group, id);
        }
        public KsInfo SetExtended(int id, int value) {
            // Only scalar acquisition options are writable here; no raw/private control guessing.
            if (id != 19) throw new ArgumentException("Only auto-exposure priority is supported by this endpoint.");
            var c = ReadKs(id);
            if (!c.Writable || (value != 0 && value != 1)) throw new ArgumentException("Unsupported setting.");
            WriteKs(id, value, 2); return ReadKs(id);
        }
        public KsInfo SetPowerLine(int value) {
            Set("video", 13, value, false); return ReadPowerLine();
        }
        public void Jog(int pan, int tilt, int durationMs, int speed) {
            if (Math.Abs(pan) > 1 || Math.Abs(tilt) > 1 || durationMs < 40 || durationMs > 400 || speed < 1 || speed > 10) throw new ArgumentException("Invalid bounded jog request.");
            var p = ReadKs(10); var t = ReadKs(11);
            if (pan != 0 && !p.Writable || tilt != 0 && !t.Writable) throw new InvalidOperationException("Relative movement is unsupported.");
            if (pan != 0 && p.Max.HasValue && speed > Math.Max(Math.Abs(p.Min.Value), Math.Abs(p.Max.Value))) throw new ArgumentException("Unsupported pan speed.");
            if (tilt != 0 && t.Max.HasValue && speed > Math.Max(Math.Abs(t.Min.Value), Math.Abs(t.Max.Value))) throw new ArgumentException("Unsupported tilt speed.");
            try {
                moving = true;
                if (pan != 0) WriteKs(10, pan * speed, 0x12); // MANUAL | RELATIVE
                if (tilt != 0) WriteKs(11, tilt * speed, 0x12);
                Thread.Sleep(durationMs);
            } finally { Stop(); }
        }
        public void Stop() {
            int first = 0;
            try { if (ReadKs(10).Writable) WriteKs(10, 0, 0x12); } catch (COMException e) { first = e.ErrorCode; }
            try { if (ReadKs(11).Writable) WriteKs(11, 0, 0x12); } catch (COMException e) { if (first == 0) first = e.ErrorCode; }
            moving = false;
            Check(first);
        }
        public void Dispose() { try { if (moving) Stop(); } finally { Release(filter); filter = null; cc = null; vp = null; ks = null; } }
    }
}
