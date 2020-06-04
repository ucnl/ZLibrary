using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO.Ports;
using System.Linq;
using System.Reflection;
using System.Text;
using UCNLDrivers;
using UCNLNav;
using UCNLNMEA;
using UCNLPhysics;
using ZLibrary.Localization;

namespace ZLibrary
{
    public enum PortState
    {
        USED,
        NOTUSED,
        UNAVAILABLE,
        OPEN,
        CLOSED,
        OK,
        DEFINED,
        TIMEOUT,
        ERROR,
        WAITING,
        UNKNOWN
    }

    public class StringEventArgs : EventArgs
    {
        #region Properties

        public string Value { get; private set; }

        #endregion

        #region Constructor

        public StringEventArgs(string str)
        {
            Value = str;
        }

        #endregion
    }

    public class GeoLocationUpdateEventArgs : EventArgs
    {
        #region Properties

        public string ItemName { get; private set; }
        public DateTime TimeStamp { get; private set; }
        public double Latitude { get; private set; }
        public double Longitude { get; private set; }
        public double Depth { get; private set; }

        #endregion

        #region Constructor

        public GeoLocationUpdateEventArgs(string name, DateTime timeStamp, double lat, double lon, double dpt)
        {
            ItemName = name;
            TimeStamp = timeStamp;
            Latitude = lat;
            Longitude = lon;
            Depth = dpt;
        }

        #endregion
    }

    public class ZResponder
    {
        #region Properties

        public ZAddress ID { get; private set; }

        public AgingValue<double> Latitude { get; private set; }
        public AgingValue<double> Longitude { get; private set; }
        public AgingValue<double> Depth { get; private set; }

        public AgingValue<double> BatVoltage { get; private set; }
        public AgingValue<double> MSR { get; private set; }
        public AgingValue<double> DPL { get; private set; }

        public AgingValue<double> Azimuth { get; private set; }
        public AgingValue<double> Distance { get; private set; }
        public AgingValue<double> DistanceProjection { get; private set; }

        public AgingValue<double> Temperature { get; private set; }
        public AgingValue<double> Pressure { get; private set; }
        public AgingValue<double> CoreTemperature { get; private set; }

        public AgingValue<double> Salinity { get; private set; }
        public AgingValue<SLP_MODE_Enum> SleepMode { get; private set; }

        public AgingValue<string> LastQueryResult { get; private set; }
        public AgingValue<bool> IsTimeout { get; private set; }

        public bool IsLocationInitializedAndNotObsolete
        {
            get { return Latitude.IsInitializedAndNotObsolete && Longitude.IsInitializedAndNotObsolete; }
        }
      
        #endregion

        #region Constructor

        public ZResponder(ZAddress id)
        {
            ID = id;

            Latitude = new AgingValue<double>(5, 300, (v) => v.ToString("F06", CultureInfo.InvariantCulture));
            Longitude = new AgingValue<double>(5, 300, (v) => v.ToString("F06", CultureInfo.InvariantCulture));

            Depth = new AgingValue<double>(5, 300, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F02} m", v));
            BatVoltage = new AgingValue<double>(5, 600, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F01} V", v));
            MSR = new AgingValue<double>(5, 300, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F01} dB", v));
            DPL = new AgingValue<double>(5, 300, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F02} Hz", v));

            Azimuth = new AgingValue<double>(5, 300, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F01}°", v));
            Distance = new AgingValue<double>(5, 300, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F02} m", v));
            DistanceProjection = new AgingValue<double>(5, 300, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F02} m", v));

            Temperature = new AgingValue<double>(5, 300, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F01} °C", v));
            Pressure = new AgingValue<double>(5, 300, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F01} mBar", v));
            CoreTemperature = new AgingValue<double>(5, 300, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F01} °C", v));

            Salinity = new AgingValue<double>(5, 300, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F00} PSU", v));
            SleepMode = new AgingValue<SLP_MODE_Enum>(5, 300, (v) => v.ToString());

            LastQueryResult = new AgingValue<string>(5, 300, (v) => v);
            IsTimeout = new AgingValue<bool>(5, 300, (v) => (v ? v.ToString() : string.Empty));
        }

        #endregion

        #region Methods

        public override string ToString()
        {
            return string.Format("RESPONDER #{0}", (int)ID);
        }

        public Dictionary<string, string> ToStrings(bool isRemoveEmptyEntries)
        {
            Dictionary<string, string> result = new Dictionary<string, string>();

            if (Latitude.IsInitialized || !isRemoveEmptyEntries)
                result.Add(ResponderDataID.LAT.ToString(), Latitude.ToString());

            if (Longitude.IsInitialized || !isRemoveEmptyEntries)
                result.Add(ResponderDataID.LON.ToString(), Longitude.ToString());

            if (Depth.IsInitialized || !isRemoveEmptyEntries)
                result.Add(ResponderDataID.DPT.ToString(), Depth.ToString());

            if (BatVoltage.IsInitialized || !isRemoveEmptyEntries)
                result.Add(ResponderDataID.BAT.ToString(), BatVoltage.ToString());

            if (MSR.IsInitialized || !isRemoveEmptyEntries)
                result.Add(ResponderDataID.MSR.ToString(), MSR.ToString());

            if (DPL.IsInitialized || !isRemoveEmptyEntries)
                result.Add(ResponderDataID.DPL.ToString(), DPL.ToString());

            if (Azimuth.IsInitialized || !isRemoveEmptyEntries)
                result.Add(ResponderDataID.AZM.ToString(), Azimuth.ToString());

            if (Distance.IsInitialized || !isRemoveEmptyEntries)
                result.Add(ResponderDataID.DST.ToString(), Distance.ToString());

            if (DistanceProjection.IsInitialized || !isRemoveEmptyEntries)
                result.Add(ResponderDataID.DSTP.ToString(), DistanceProjection.ToString());

            if (Temperature.IsInitialized || !isRemoveEmptyEntries)
                result.Add(ResponderDataID.TMP.ToString(), Temperature.ToString());

            if (Pressure.IsInitialized || !isRemoveEmptyEntries)
                result.Add(ResponderDataID.PRS.ToString(), Pressure.ToString());

            if (CoreTemperature.IsInitialized || !isRemoveEmptyEntries)
                result.Add(ResponderDataID.CTM.ToString(), CoreTemperature.ToString());

            if (Salinity.IsInitialized || !isRemoveEmptyEntries)
                result.Add(ResponderDataID.STY.ToString(), Salinity.ToString());

            if (SleepMode.IsInitialized || !isRemoveEmptyEntries)
                result.Add(ResponderDataID.STY.ToString(), SleepMode.ToString());

            if (LastQueryResult.IsInitialized || !isRemoveEmptyEntries)
                result.Add(ResponderDataID.LQR.ToString(), LastQueryResult.ToString());

            if ((IsTimeout.IsInitialized && IsTimeout.Value) || !isRemoveEmptyEntries)
                result.Add(ResponderDataID.TMO.ToString(), IsTimeout.ToString());

            return result;
        }

        public Dictionary<ResponderDataID, IAging> ToIAgings()
        {
            Dictionary<ResponderDataID, IAging> result = new Dictionary<ResponderDataID, IAging>();
            result.Add(ResponderDataID.LAT, Latitude);
            result.Add(ResponderDataID.LON, Longitude);
            result.Add(ResponderDataID.DPT, Depth);
            result.Add(ResponderDataID.BAT, BatVoltage);
            result.Add(ResponderDataID.MSR, MSR);
            result.Add(ResponderDataID.DPL, DPL);
            result.Add(ResponderDataID.AZM, Azimuth);
            result.Add(ResponderDataID.DST, Distance);
            result.Add(ResponderDataID.DSTP, DistanceProjection);
            result.Add(ResponderDataID.TMP, Temperature);
            result.Add(ResponderDataID.PRS, Pressure);
            result.Add(ResponderDataID.CTM, CoreTemperature);
            result.Add(ResponderDataID.STY, Salinity);
            result.Add(ResponderDataID.SLP, SleepMode);
            result.Add(ResponderDataID.LQR, LastQueryResult);
            result.Add(ResponderDataID.TMO, IsTimeout);

            return result;
        }

        #endregion
    }

    public class ZCore : IDisposable
    {
        #region Properties

        SerialPort outPort;
        SerialPortsPool auxPorts;
        ZPort zport;

        NMEAMultipleListener nmeaListener;
        PrecisionTimer timer;

        Dictionary<int, string> portNamesDictionary;

        bool disposed = false;

        Dictionary<ZAddress, ZResponder> responders;
        public IReadOnlyDictionary<ZAddress, ZResponder> Responders
        {
            get { return (IReadOnlyDictionary<ZAddress, ZResponder>)responders; }
        }

        public AgingValue<double> Latitude { get; private set; }
        public AgingValue<double> Longitude { get; private set; }
        public AgingValue<double> Depth { get; private set; }
        public AgingValue<double> Azimuth { get; private set; }
        public AgingValue<double> Pitch { get; private set; }
        public AgingValue<double> Roll { get; private set; }

        public AgingValue<double> Temperature { get; private set; }
        public AgingValue<double> SpeedKmh { get; private set; }
        public AgingValue<double> VTGTrack { get; private set; }
        public AgingValue<DateTime> GNSSTime { get; private set; }


        public string StationSystemDescription { get; private set; }
        public string StationCoreDescription { get; private set; }
        public string StationSerialNumber { get; private set; }

        public ZAddress OutPortResponderAddress { get; private set; }

        public bool IsRunning
        {
            get { return zport.IsOpen; }
        }

        public bool isDepthLessThanAllowed { get; private set; }

        bool isAUXPortsUsed = false;
        bool isOutPortUsed = false;


        PortState zPortState = PortState.NOTUSED;
        public PortState ZPortState
        {
            get { return zPortState; }
            private set
            {
                if (value != zPortState)
                {
                    zPortState = value;
                    ZPortStateChangedEventHandler.Rise(this, new EventArgs());
                }
            }
        }

        PortState gnssPortState = PortState.NOTUSED;
        public PortState GNSSPortState
        {
            get { return gnssPortState; }
            private set
            {
                if (value != gnssPortState)
                {
                    gnssPortState = value;
                    GNSSPortStateChangedEventHandler.Rise(this, new EventArgs());
                }
            }
        }

        PortState hdgPortState = PortState.NOTUSED;
        public PortState HDGPortState
        {
            get { return hdgPortState; }
            private set
            {
                if (value != hdgPortState)
                {
                    hdgPortState = value;
                    HDGPortStateChangedEventHandler.Rise(this, new EventArgs());
                }
            }
        }

        PortState outPortState = PortState.NOTUSED;
        public PortState OutPortState
        {
            get { return outPortState; }
            private set
            {
                if (value != outPortState)
                {
                    outPortState = value;
                    OutPortStateChangedEventHandler.Rise(this, new EventArgs());
                }
            }
        }

        bool isAutoQuery = false;
        public bool IsAutoQuery
        {
            get { return isAutoQuery; }
            set
            {
                if (zport.IsOpen)
                {
                    isAutoQuery = value;
                    if (isAutoQuery && !zport.IsBusy)
                        zport_IsBusyStateChanged(zport, new EventArgs());
                }
                else
                    throw new InvalidOperationException(LocStringManager.ZCoreShouldBeStartedFirst_str);
            }
        }

        bool isStationDeviceInfoUpdated = false;
        public bool IsStationDeviceInfoUpdated
        {
            get { return isStationDeviceInfoUpdated; }
            private set
            {
                if (isStationDeviceInfoUpdated != value)
                {
                    isStationDeviceInfoUpdated = value;
                    IsStationDeviceInfoUpdatedChangedEventHandler.Rise(this, new EventArgs());
                }
            }
        }

        bool isStationSalinityUpdated = false;
        bool isStationMaxDistanceUpdated = false;

        double maxDistance_m;
        public double MaxDistance_m
        {
            get { return maxDistance_m; }
            set
            {
                if (value.IsInRangeInclusive(ZMA.MinMaxDistance_m, ZMA.MaxMaxDistance_m))
                {
                    maxDistance_m = value;
                    isStationMaxDistanceUpdated = false;
                }
                else
                    throw new ArgumentOutOfRangeException();
            }
        }

        double waterSalinity_PSU;
        public double WaterSalinity_PSU
        {
            get { return waterSalinity_PSU; }
            set
            {
                if (value.IsInRangeInclusive(ZMA.MinSalinityPPM, ZMA.MaxSalinityPPM))
                {
                    waterSalinity_PSU = value;
                    isStationSalinityUpdated = false;
                }
                else
                    throw new ArgumentOutOfRangeException();
            }
        }

        public bool IsRoughDepth { get; set; }

        double stationAdjustAngle = 0;
        public double StationAdjustAngle
        {
            get { return stationAdjustAngle; }
            set
            {
                if (value.IsInRangeInclusive(0, 360))
                    stationAdjustAngle = value;
                else
                    throw new ArgumentOutOfRangeException();
            }
        }

        double gnss2StationDistance_m;
        double stationDelta_deg;

        double stationOffsetX;
        public double StationOffsetX
        {
            get { return stationOffsetX; }
            set
            {
                stationOffsetX = value;
                gnss2StationDistance_m = Math.Sqrt(stationOffsetX * stationOffsetX + stationOffsetY * stationOffsetY);
                var delta = Math.Asin(stationOffsetY / gnss2StationDistance_m);

                if (double.IsNaN(delta))
                    stationDelta_deg = 0.0;
                else
                    stationDelta_deg = delta;
            }
        }

        double stationOffsetY;
        public double StationOffsetY
        {
            get { return stationOffsetY; }
            set
            {
                stationOffsetY = value;
                gnss2StationDistance_m = Math.Sqrt(stationOffsetX * stationOffsetX + stationOffsetY * stationOffsetY);
                var delta = Math.Asin(stationOffsetY / gnss2StationDistance_m);

                if (double.IsNaN(delta))
                    stationDelta_deg = 0.0;
                else
                    stationDelta_deg = Algorithms.Rad2Deg(delta);
            }
        }

        public bool IsHeadingFixed { get; set; }
        public bool IsUseVTGAsHeadingSource { get; set; }

        bool isSoundSpeedToSet = false;
        bool isSoundSpeedUpdated = true;
        double speedOfSound = PHX.PHX_FWTR_SOUND_SPEED_MPS;
        public double SpeedOfSound
        {
            get { return speedOfSound; }
            set
            {
                if ((value >= PHX.PHX_FWTR_SOUND_SPEED_MPS_MIN) && (value <= PHX.PHX_FWTR_SOUND_SPEED_MPS_MAX))
                {
                    speedOfSound = value;
                    isSoundSpeedToSet = true;
                    isSoundSpeedUpdated = false;
                }
                else
                    throw new ArgumentOutOfRangeException(string.Format("Sound speed value should be in a range {0:F00}..{1:F00} m/s",
                        PHX.PHX_FWTR_SOUND_SPEED_MPS_MIN, PHX.PHX_FWTR_SOUND_SPEED_MPS_MAX));
            }
        }

        public bool IsSaveAUXLog { get; set; }

        ZAddress prevAddr = ZAddress.INVALID;
        bool isHDTPresent = false;

        string action;

        double stationDepthAdjust = 0.0;

        bool isStationRemoteQueryExSupported = true;

        Dictionary<int, string> aux_names;

        #endregion

        #region Constructor

        public ZCore(SerialPortSettings zPortSettings)
        {
            LocStringManager.Init("ZLibrary.Localization.LocStringResources", Assembly.GetExecutingAssembly());

            IsRoughDepth = false;
            maxDistance_m = 2000;            

            OutPortResponderAddress = ZAddress.Responder_1;

            responders = new Dictionary<ZAddress, ZResponder>();

            Depth = new AgingValue<double>(2, 5, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F02} m", v));
            Pitch = new AgingValue<double>(2, 5, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F01}°", v));
            Roll = new AgingValue<double>(2, 5, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F01}°", v));
            Temperature = new AgingValue<double>(2, 5, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F01} °C", v));
            StationSystemDescription = string.Empty;
            StationCoreDescription = string.Empty;
            StationSerialNumber = string.Empty;

            Latitude = new AgingValue<double>(2, 5, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F06}°", v));
            Longitude = new AgingValue<double>(2, 5, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F06}°", v));
            Azimuth = new AgingValue<double>(2, 5, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F01}°", v));

            SpeedKmh = new AgingValue<double>(2, 5, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F02} km/h", v));
            VTGTrack = new AgingValue<double>(2, 5, (v) => string.Format(CultureInfo.InvariantCulture, "{0:F01}°", v));
            GNSSTime = new AgingValue<DateTime>(2, 5, (v) => v.ToShortTimeString());

            zport = new ZPort(zPortSettings);
            zport.LogEventHandler += (o, e) => LogEventHandler.Rise(o, e);
            zport.PortErrorEventHandler += (o, e) =>
                {
                    ZPortState = PortState.ERROR;
                    LogEventHandler.Rise(o, new LogEventArgs(LogLineType.ERROR, string.Format("{0} in {1}", e.EventType.ToString(), zport.PortName)));
                };
            zport.DeviceInfoUpdatedEventHandler += new EventHandler(zport_DeviceInfoUpdated);
            zport.IsBusyChangedEventHandler += new EventHandler(zport_IsBusyStateChanged);
            zport.LocalDataReceivedEventHandler += new EventHandler<LocDataUpdatedEventArgs>(zport_LocalDataReceived);
            zport.RemoteResponseEventHandler += new EventHandler<RemoteResponseEventArgs>(zport_RemoteResponseReceived);
            zport.RemoteTimeoutEventHandler += new EventHandler<RemoteTimeoutEventArgs>(zport_RemoteTimeout);
            zport.SystemOrientationUpdatedEventHandler += new EventHandler(zport_SystemOrientationUpdated);
            zport.SystemStateUpdatedEventHandler += new EventHandler(zport_SystemStateUpdated);
            zport.SystemTimeoutEventHandler += (o, e) =>
                {
                    ZPortState = PortState.TIMEOUT;
                    LogEventHandler.Rise(o, new LogEventArgs(LogLineType.ERROR, string.Format("TIMEOUT in {0}", zport.PortName)));
                };
            zport.StationACKEventHandler += new EventHandler<StationACKEventArgs>(port_StationACKReceived);

            nmeaListener = new NMEAMultipleListener();
            nmeaListener.LogEventHandler += (o, e) => LogEventHandler.Rise(o, e);
            nmeaListener.HDGSentenceReceived += new EventHandler<HDGMessageEventArgs>(nmeaListener_HDGReceived);
            nmeaListener.HDTSentenceReceived += new EventHandler<HDTMessageEventArgs>(nmeaListener_HDTReceived);
            nmeaListener.VTGSentenceReceived += new EventHandler<VTGMessageEventArgs>(nmeaListener_VTGReceived);
            nmeaListener.RMCSentenceReceived += new EventHandler<RMCMessageEventArgs>(nmeaListener_RMCReceived);
            nmeaListener.NMEAIncomingMessageReceived += (o, e) => 
            { 
                if (IsSaveAUXLog) 
                {
                    string aux_name = portNamesDictionary.ContainsKey(e.SourceID) ? portNamesDictionary[e.SourceID] : string.Format("AUX {0}", e.SourceID.ToString());
                    LogEventHandler.Rise(o, new LogEventArgs(LogLineType.INFO, string.Format("{0} >> {1}", aux_name, e.Message))); 
                } 
            };

            timer = new PrecisionTimer();
            timer.Mode = Mode.Periodic;
            timer.Period = 1000;
            timer.Tick += new EventHandler(timer_Tick);

            ZPortState = PortState.USED;
        }

        #endregion

        #region Methods

        #region Private

        private void ProcessResponder(ZAddress address, double rAzimuth)
        {
            // Calculate slant range projection on the water surface
            if (Depth.IsInitialized && Responders[address].Depth.IsInitialized && responders[address].Distance.IsInitialized)
            {

                var slantProjection = GetSlantRangeProjection(responders[address].Distance.Value, Depth.Value, responders[address].Depth.Value);
                if (!double.IsNaN(slantProjection))
                    responders[address].DistanceProjection.Value = slantProjection;
            }
            else
            {
                LogEventHandler.Rise(this,
                    new LogEventArgs(LogLineType.INFO, "Unable to calculate slant range projection"));
            }


            if (Azimuth.IsInitialized && !Azimuth.IsObsolete)
            {
                var antTrueAzimuth = Algorithms.Wrap360(Azimuth.Value + stationAdjustAngle);
                responders[address].Azimuth.Value = Algorithms.Wrap360(rAzimuth + antTrueAzimuth);

                if (responders[address].DistanceProjection.IsInitialized)
                {
                    if (Latitude.IsInitialized && !Latitude.IsObsolete)
                    {
                        double stLat = Latitude.Value;
                        double stLon = Longitude.Value;

                        double spLat = Algorithms.Deg2Rad(Latitude.Value);
                        double spLon = Algorithms.Deg2Rad(Longitude.Value);
                        double fwd_az = Algorithms.Deg2Rad(Algorithms.Wrap360(Azimuth.Value + stationDelta_deg));
                        double rev_az = fwd_az;
                        int its = 0;

                        if (!Algorithms.VincentyDirect(spLat, spLon, fwd_az, gnss2StationDistance_m,
                            Algorithms.WGS84Ellipsoid, 10E-12, 200, out stLat, out stLon, out rev_az, out its))
                        {
                            Algorithms.HaversineDirect(spLat, spLon, gnss2StationDistance_m, fwd_az, Algorithms.WGS84Ellipsoid.MajorSemiAxis_m,
                                out stLat, out stLon);
                        }

                        double bLat = stLat;
                        double bLon = stLon;
                        fwd_az = Algorithms.Deg2Rad(responders[address].Azimuth.Value);
                        if (!Algorithms.VincentyDirect(stLat, stLon, fwd_az, responders[address].DistanceProjection.Value,
                            Algorithms.WGS84Ellipsoid, 10E-12, 200, out bLat, out bLon, out rev_az, out its))
                        {
                            Algorithms.HaversineDirect(stLat, stLon, gnss2StationDistance_m, fwd_az, Algorithms.WGS84Ellipsoid.MajorSemiAxis_m,
                                out bLat, out bLon);
                        }

                        bLat = Algorithms.Rad2Deg(bLat);
                        bLon = Algorithms.Rad2Deg(bLon);

                        responders[address].Latitude.Value = bLat;
                        responders[address].Longitude.Value = bLon;

                        GeoLocationUpdatedEventHandler.Rise(this,
                            new GeoLocationUpdateEventArgs(address.ToString(), DateTime.Now, responders[address].Latitude.Value, responders[address].Longitude.Value, responders[address].Depth.Value));

                        if (isOutPortUsed && (OutPortResponderAddress == address) &&
                            ((OutPortState == PortState.OPEN) || (OutPortState == PortState.OK)))
                        {
                            WriteOutData(bLat, bLon, responders[address].Depth.Value, responders[address].Azimuth.Value);
                        }
                    }
                    else
                    {
                        LogEventHandler.Rise(this,
                            new LogEventArgs(LogLineType.INFO, "Warning: unable to calculate absoulute position - GNSS data not available"));
                    }
                }
                else
                {
                    LogEventHandler.Rise(this,
                            new LogEventArgs(LogLineType.INFO, "Warning: unable to calculate absoulute position - cannot estimate slant range projection"));
                }
            }
            else
            {
                responders[address].Azimuth.Value = Algorithms.Wrap360(rAzimuth + stationAdjustAngle);
                LogEventHandler.Rise(this,
                            new LogEventArgs(LogLineType.INFO, "Warning: unable to calculate absoulute heading - heading data not available"));
            }
        }

        private void ProcessRemoteQueries()
        {
            var responderAddress = GetNextResponder(prevAddr);

            if (responderAddress != ZAddress.INVALID)
            {
                CDS_NODE_CMD_Enum cmdID = CDS_NODE_CMD_Enum.CDS_DPT_GET;
                if (IsRoughDepth)
                    cmdID = CDS_NODE_CMD_Enum.CDS_DPT_XM_GET;

                if (!responders[responderAddress].Salinity.IsInitialized)
                {
                    cmdID = (CDS_NODE_CMD_Enum)((int)CDS_NODE_CMD_Enum.CDS_STY_SET_0 + (int)waterSalinity_PSU);
                }
                else if ((!responders[responderAddress].Temperature.IsInitialized) ||
                         (responders[responderAddress].Temperature.Age.TotalSeconds > ZMA.Temp_Periodicity))
                {
                    cmdID = CDS_NODE_CMD_Enum.CDS_PTS_TMP_GET;
                }
                else if ((!responders[responderAddress].BatVoltage.IsInitialized) ||
                         ((responders[responderAddress].BatVoltage.Age.TotalSeconds > ZMA.Bat_Periodicity) &&
                         ((string.IsNullOrEmpty(responders[responderAddress].BatVoltage.AccessTag)))))
                {
                    cmdID = CDS_NODE_CMD_Enum.CDS_BAT_CHG_GET;
                }

                if (cmdID == CDS_NODE_CMD_Enum.CDS_DPT_GET)                    
                {
                    if (responders[responderAddress].Azimuth.IsInitialized)
                    {
                        if (isStationRemoteQueryExSupported)
                        {
                            if (responders[responderAddress].IsLocationInitializedAndNotObsolete)
                            {
                                double sp_lat_rad = Algorithms.Deg2Rad(Latitude.Value);
                                double sp_lon_rad = Algorithms.Deg2Rad(Longitude.Value);
                                double ep_lat_rad = Algorithms.Deg2Rad(responders[responderAddress].Latitude.Value);
                                double ep_lon_rad = Algorithms.Deg2Rad(responders[responderAddress].Longitude.Value);
                                double rev_az = Algorithms.Rad2Deg(Algorithms.Wrap2PI(Algorithms.HaversineFinalBearing(sp_lat_rad, sp_lon_rad, ep_lat_rad, ep_lon_rad)));
                                zport.QueryRemoteEx(responderAddress, cmdID, rev_az);
                            }
                            else if (IsHeadingFixed)
                            {
                                double rev_az = Algorithms.Wrap360(responders[responderAddress].Azimuth.Value + 180.0);
                                zport.QueryRemoteEx(responderAddress, cmdID, rev_az);
                            }
                            else
                            {
                                zport.QueryRemote(responderAddress, cmdID);
                            }
                        }
                        else
                        {
                            zport.QueryRemote(responderAddress, cmdID);
                        }                        
                    }
                    else
                    {
                        zport.QueryRemote(responderAddress, cmdID);
                    }

                    OnActionInit(string.Format("? {0} >> {1}", cmdID, responderAddress));
                }
                else
                {
                    zport.QueryRemote(responderAddress, cmdID);
                    OnActionInit(string.Format("? {0} >> {1}", cmdID, responderAddress));
                }

                prevAddr = responderAddress;
            }
        }

        private double GetSlantRangeProjection(double slantRange_m, double dpt1, double dpt2)
        {
            double ddpt = Math.Abs(dpt1 - dpt2);
            if (slantRange_m < ddpt)
                return double.NaN;
            else
                return Math.Sqrt(slantRange_m * slantRange_m - ddpt * ddpt);
        }

        private void WriteOutData(double bLat, double bLon, double bDpt, double bAzm)
        {            
            string latCardinal, lonCardinal;

            if (bLat > 0) latCardinal = "North";
            else latCardinal = "South";

            if (bLon > 0) lonCardinal = "East";
            else lonCardinal = "West";

            StringBuilder emuString = new StringBuilder();

            #region RMC

            emuString.Append(NMEAParser.BuildSentence(TalkerIdentifiers.GP, SentenceIdentifiers.RMC, new object[] 
                    {
                        DateTime.Now, 
                        "Valid", 
                        bLat, latCardinal,
                        bLon, lonCardinal,
                        SpeedKmh.Value / 0.5144,
                        null, // track true
                        DateTime.Now,
                        null, // magnetic variation
                        null, // magnetic variation direction
                        "A",
                    }));

            #endregion

            #region GGA

            if (bLat > 0) latCardinal = "N";
            else latCardinal = "S";

            if (bLon > 0) lonCardinal = "E";
            else lonCardinal = "W";

            emuString.Append(NMEAParser.BuildSentence(TalkerIdentifiers.GP, SentenceIdentifiers.GGA, new object[]
                            {
                                DateTime.Now,
                                bLat, latCardinal,
                                bLon, lonCardinal,
                                "GPS fix",
                                4,
                                null,
                                -bDpt,
                                "M",
                                null,
                                "M",
                                null,
                                null
                            }));

            #endregion

            #region HDT

            emuString.Append(NMEAParser.BuildSentence(TalkerIdentifiers.GP, SentenceIdentifiers.HDT, new object[]
                {
                    bAzm,
                    "T"
                }));

            #endregion

            var emuStr = emuString.ToString();

            try
            {
                var bytes = Encoding.ASCII.GetBytes(emuStr);
                outPort.Write(bytes, 0, bytes.Length);

                WrittenToOutputPortEventHandler.Rise(this, new StringEventArgs(emuStr));
                LogEventHandler.Rise(this, new LogEventArgs(LogLineType.INFO, string.Format("{0} (OUT) << {1}", outPort.PortName, emuStr)));

            }
            catch (Exception ex)
            {
                LogEventHandler.Rise(this, new LogEventArgs(LogLineType.ERROR, ex));
            }
        }

        private ZAddress GetNextResponder(ZAddress address)
        {
            ZAddress result = ZAddress.INVALID;

            //if (responders.ContainsKey(address))
            {
                var keys = responders.Keys.ToList();
                var nIdx = (keys.IndexOf(address) + 1) % keys.Count;
                result = keys[nIdx];
            }

            return result;
        }

        private void OnActionInit(string actionDescription)
        {
            action = actionDescription;
            RemoteActionProgessEventHandler.Rise(this, new StringEventArgs(action));
        }

        private void OnActionResult(string resultDescription)
        {
            RemoteActionProgessEventHandler.Rise(this, new StringEventArgs(string.Format("{0} -> {1}", action, resultDescription)));
        }

        #endregion

        #region Public
      
        public void Start()
        {
            ZPortState = PortState.USED;

            try
            {
                isStationDeviceInfoUpdated = false;
                isStationSalinityUpdated = false;
                zport.Open();
                ZPortState = PortState.OPEN;


                zport.QueryLocalDataGet(LOC_DATA_ID.LOC_DATA_DEVICE_INFO);
            }
            catch (Exception ex)
            {
                LogEventHandler.Rise(this, new LogEventArgs(LogLineType.CRITICAL, ex));
                ZPortState = PortState.UNAVAILABLE;
            }

            if (isAUXPortsUsed)
            {
                try
                {
                    auxPorts.Open();
                    GNSSPortState = PortState.OPEN;
                    HDGPortState = PortState.OPEN;
                    isHDTPresent = false;
                }
                catch (Exception ex)
                {
                    LogEventHandler.Rise(this, new LogEventArgs(LogLineType.ERROR, ex));
                    GNSSPortState = PortState.UNAVAILABLE;
                    HDGPortState = PortState.UNAVAILABLE;
                }
            }

            if (isOutPortUsed)
            {
                try
                {
                    outPort.Open();
                    OutPortState = PortState.OPEN;
                }
                catch (Exception ex)
                {
                    LogEventHandler.Rise(this, new LogEventArgs(LogLineType.ERROR, ex));
                    OutPortState = PortState.UNAVAILABLE;
                }
            }

            timer.Start();
        }

        public void Stop()
        {
            IsAutoQuery = false;

            timer.Stop();

            try
            {
                zport.Close();
            }
            catch (Exception ex)
            {
                LogEventHandler.Rise(this, new LogEventArgs(LogLineType.ERROR, ex));
            }

            ZPortState = PortState.CLOSED;

            if (isAUXPortsUsed)
            {
                try
                {
                    auxPorts.Close();
                }
                catch (Exception ex)
                {
                    LogEventHandler.Rise(this, new LogEventArgs(LogLineType.ERROR, ex));
                }

                GNSSPortState = PortState.CLOSED;
                HDGPortState = PortState.CLOSED;
            }

            if (isOutPortUsed)
            {
                try
                {
                    outPort.Close();
                    OutPortState = PortState.CLOSED;
                }
                catch (Exception ex)
                {
                    LogEventHandler.Rise(this, new LogEventArgs(LogLineType.ERROR, ex));
                }
            }
        }

        public void RemoteCommand(ZAddress address, CDS_NODE_CMD_Enum cmdID)
        {
            //if (!ZMA.CDS_IS_USR_CMD(cmdID))
            //    throw new ArgumentOutOfRangeException("cmdID");

            if (!isAutoQuery)
            {
                if (ZPortState == PortState.OK)
                {
                    zport.QueryRemote(address, cmdID);
                    OnActionInit(string.Format("? {0} >> {1}", cmdID, address));
                }
                else
                    throw new InvalidOperationException(string.Format("{0} ({1})", 
                        LocStringManager.UnableToProcessRequestInCurrentPortState_str,
                        zPortState));
            }
            else
                throw new InvalidOperationException(LocStringManager.UnableToProcessRequestWhileIsAutoQueryModeIsSelected_str); //"Unable to procees request while IsAutoQuery mode is selected");
        }

        public void RemoteAddressChange(ZAddress currentAddress, ZAddress newAddress)
        {
            RemoteCommand(currentAddress, (CDS_NODE_CMD_Enum)(((int)CDS_NODE_CMD_Enum.CDS_SET_ADDR_01) + ((int)newAddress) - 1));
        }

        public string GetStationStateDescription(StationDataID dataIDs, bool discardUninitializedEntries)
        {
            StringBuilder sb = new StringBuilder();

            if (dataIDs.HasFlag(StationDataID.TRX))
                sb.AppendFormat("TRX: {0}\r\n", zport.TRX_State);

            if (dataIDs.HasFlag(StationDataID.DPT) && (Depth.IsInitialized || !discardUninitializedEntries))
                sb.AppendFormat("DPT: {0}\r\n", Depth);

            if (dataIDs.HasFlag(StationDataID.TMP) && (Temperature.IsInitialized || !discardUninitializedEntries))
                sb.AppendFormat("TMP: {0}\r\n", Temperature);

            if (dataIDs.HasFlag(StationDataID.ROL) && (Roll.IsInitialized || !discardUninitializedEntries))
                sb.AppendFormat("ROL: {0}\r\n", Roll);

            if (dataIDs.HasFlag(StationDataID.PTC) && (Pitch.IsInitialized || !discardUninitializedEntries))
                sb.AppendFormat("PTC: {0}\r\n", Pitch);

            if (dataIDs.HasFlag(StationDataID.LAT) && (Latitude.IsInitialized || !discardUninitializedEntries))
                sb.AppendFormat("LAT: {0}\r\n", Latitude);

            if (dataIDs.HasFlag(StationDataID.LON) && (Longitude.IsInitialized || !discardUninitializedEntries))
                sb.AppendFormat("LON: {0}\r\n", Longitude);

            if (dataIDs.HasFlag(StationDataID.AZM) && (Azimuth.IsInitialized || !discardUninitializedEntries))
                sb.AppendFormat("AZM: {0}\r\n", Azimuth);

            if (dataIDs.HasFlag(StationDataID.SPD) && (SpeedKmh.IsInitialized || !discardUninitializedEntries))
                sb.AppendFormat("SPD: {0}\r\n", SpeedKmh);

            if (dataIDs.HasFlag(StationDataID.VTG) && (VTGTrack.IsInitialized || !discardUninitializedEntries))
                sb.AppendFormat("VTG: {0}\r\n", VTGTrack);

            return sb.ToString();
        }

        public void AddResponder(ZAddress address)
        {
            responders.Add(address, new ZResponder(address));
        }

        public void AUXSourcesInit(SerialPortSettings[] auxPortsSettings)
        {
            if (!isAUXPortsUsed)
            {               
                auxPorts = new SerialPortsPool(auxPortsSettings);
                portNamesDictionary = new Dictionary<int, string>();
                foreach (var item in auxPortsSettings)                
                    portNamesDictionary.Add(item.PortName.GetHashCode(), item.PortName);

                auxPorts.DataReceived += (o, e) => nmeaListener.ProcessIncoming(e.PortName.GetHashCode(), Encoding.ASCII.GetString(e.Data));
                auxPorts.ErrorReceived += (o, e) => LogEventHandler.Rise(o, new LogEventArgs(LogLineType.ERROR, string.Format("{0} in {1}", e.PortName, e.EventType.ToString())));
                auxPorts.LogEventHandler += (o, e) => LogEventHandler.Rise(o, e);

                GNSSPortState = PortState.USED;
                HDGPortState = PortState.USED;
                isAUXPortsUsed = true;
            }
        }

        public void OutPortInit(SerialPortSettings outPortSettings, ZAddress responderAddress)
        {
            if (!isOutPortUsed)
            {
                if (responderAddress != ZAddress.INVALID)
                {
                    outPort = new SerialPort(outPortSettings.PortName, (int)outPortSettings.PortBaudRate,
                    outPortSettings.PortParity, (int)outPortSettings.PortDataBits, outPortSettings.PortStopBits);

                    outPort.WriteTimeout = 500;

                    OutPortResponderAddress = responderAddress;
                    OutPortState = PortState.USED;
                    isOutPortUsed = true;
                }
                else
                    throw new ArgumentOutOfRangeException("responderAddress");
            }
            else
                throw new InvalidOperationException(LocStringManager.OutputPortIsAlreadyInitialized_str); // "Output port is already initialized");
        }

        public void SetActualDepth(double actualDepthM)
        {
            if (Depth.IsInitializedAndNotObsolete)
            {
                stationDepthAdjust = actualDepthM - (Depth.Value - stationDepthAdjust);
                LogEventHandler.Rise(this, new LogEventArgs(LogLineType.ERROR, string.Format("SetActualDepth({0:F03}), Station Depth = {1:F03}, Depth adjustment = {2:F03}", actualDepthM, Depth.Value, stationDepthAdjust)));
            }
        }


        public void EmulationInput(string eString)
        {
            LogEventHandler.Rise(this, new LogEventArgs(LogLineType.INFO, string.Format("EMU_INPUT: {0}", eString)));

            if (eString.StartsWith("$PZMA"))
                zport.DataReceived_Emul(eString);
            else
                nmeaListener.ProcessIncoming(0, eString);
        }

        #endregion

        #endregion

        #region Handlers

        #region timer

        private void timer_Tick(object sender, EventArgs e)
        {
            if (isAUXPortsUsed)
            {
                if ((!Latitude.IsInitialized) || (Latitude.Age.TotalSeconds > 2))
                    GNSSPortState = PortState.TIMEOUT;

                if ((!Azimuth.IsInitialized) || (Azimuth.Age.TotalSeconds > 2))
                {
                    isHDTPresent = false;
                    HDGPortState = PortState.TIMEOUT;
                }
            }

            On1PPSEventHandler.Rise(this, e);
        }

        #endregion

        #region zport

        private void zport_DeviceInfoUpdated(object sender, EventArgs e)
        {
            StationSystemDescription = string.Format("{0} v{1}", zport.SystemMoniker, zport.SystemVersion);
            StationCoreDescription = string.Format("{0} v{1}", zport.CoreMoniker, zport.CoreVersion);
            StationSerialNumber = zport.Serial;
            IsStationDeviceInfoUpdated = true;
            ZPortState = PortState.DEFINED;
            StationUpdatedEventHandler.Rise(this, new EventArgs());
        }

        private void zport_IsBusyStateChanged(object sender, EventArgs e)
        {
            if (!zport.IsBusy)
            {
                if (!isStationDeviceInfoUpdated)
                {
                    zport.QueryLocalDataGet(LOC_DATA_ID.LOC_DATA_DEVICE_INFO);
                }
                else if (!isStationMaxDistanceUpdated)
                {
                    zport.QueryLocalDataSet(LOC_DATA_ID.LOC_DATA_MAX_DIST, maxDistance_m);
                }                
                else if (!isStationSalinityUpdated)
                {
                    zport.QueryLocalDataSet(LOC_DATA_ID.LOC_DATA_SALINITY, waterSalinity_PSU);
                }
                else if (isSoundSpeedToSet && !isSoundSpeedUpdated)
                {
                    zport.QueryLocalDataSet(LOC_DATA_ID.LOC_DATA_SOUNDSPEED, speedOfSound);
                    isSoundSpeedToSet = false;
                }
            }

            ZPortState = PortState.OK;
        }

        private void zport_LocalDataReceived(object sender, LocDataUpdatedEventArgs e)
        {
            switch (e.DataID)
            {
                case LOC_DATA_ID.LOC_DATA_MAX_DIST:
                    {
                        isStationMaxDistanceUpdated = true;
                        break;
                    }
                case LOC_DATA_ID.LOC_DATA_SALINITY:
                    {
                        isStationSalinityUpdated = true;
                        break;
                    }
                case LOC_DATA_ID.LOC_DATA_SOUNDSPEED:
                    {
                        isSoundSpeedUpdated = (e.DataValue == speedOfSound);
                        break;
                    }
            }

            ZPortState = PortState.OK;
            LocalDataUpdatedEventHandler.Rise(this, e);
        }

        private void zport_RemoteResponseReceived(object sender, RemoteResponseEventArgs e)
        {
            if (!responders.ContainsKey(e.Address))
                responders.Add(e.Address, new ZResponder(e.Address));

            responders[e.Address].IsTimeout.Value = false;

            if (!double.IsNaN(e.Distance))
                responders[e.Address].Distance.Value = e.Distance;

            responders[e.Address].DPL.Value = e.Dpl;
            responders[e.Address].MSR.Value = Math.Abs(e.MSR);

            switch (e.CommandID)
            {
                case CDS_NODE_CMD_Enum.CDS_PING:
                    {
                        break;
                    }
                case CDS_NODE_CMD_Enum.CDS_DPT_GET:
                    {
                        if (responders[e.Address].Depth.IsInitialized)
                        {
                            double dpt = responders[e.Address].Depth.Value;

                            if (e.DFlag == 1)
                                responders[e.Address].Depth.Value = e.Value;
                            else
                                responders[e.Address].Depth.Value = e.Value + (dpt - (int)dpt);
                        }
                        else
                        {
                            responders[e.Address].Depth.Value = e.Value;
                        }
                        break;
                    }

                case CDS_NODE_CMD_Enum.CDS_DPT_XM_GET:
                    {
                        responders[e.Address].Depth.Value = e.Value;
                        break;
                    }
                case CDS_NODE_CMD_Enum.CDS_STY_SET_0:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_1:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_2:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_3:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_4:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_5:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_6:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_7:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_8:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_9:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_10:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_11:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_12:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_13:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_14:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_15:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_16:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_17:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_18:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_19:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_20:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_21:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_22:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_23:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_24:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_25:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_26:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_27:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_28:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_29:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_30:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_31:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_32:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_33:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_34:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_35:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_36:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_37:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_38:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_39:
                case CDS_NODE_CMD_Enum.CDS_STY_SET_40:
                    {
                        // beacon has set the salinity
                        responders[e.Address].Salinity.Value = ((int)e.CommandID - (int)CDS_NODE_CMD_Enum.CDS_STY_SET_0);
                        responders[e.Address].LastQueryResult.Value = e.CommandID.ToString();
                        break;
                    }

                case CDS_NODE_CMD_Enum.CDS_SLP_SET_59_60:
                case CDS_NODE_CMD_Enum.CDS_SLP_SET_58_60:
                case CDS_NODE_CMD_Enum.CDS_SLP_SET_56_60:
                case CDS_NODE_CMD_Enum.CDS_SLP_SET_52_60:
                case CDS_NODE_CMD_Enum.CDS_SLP_SET_50_60:
                case CDS_NODE_CMD_Enum.CDS_SLP_SET_40_60:
                case CDS_NODE_CMD_Enum.CDS_SLP_SET_30_60:
                case CDS_NODE_CMD_Enum.CDS_SLP_SET_20_60:
                case CDS_NODE_CMD_Enum.CDS_SLP_SET_10_60:
                case CDS_NODE_CMD_Enum.CDS_SLP_SET_NEVER:
                    {
                        // beacon has set the sleep mode
                        responders[e.Address].LastQueryResult.Value = e.CommandID.ToString();
                        break;
                    }

                case CDS_NODE_CMD_Enum.CDS_BAT_CHG_GET:
                    {
                        responders[e.Address].BatVoltage.Value = e.Value;
                        if (e.Value > ZMA.MaxBatChargeV)
                            responders[e.Address].BatVoltage.AccessTag = ((CDS_NODE_CMD_Enum)Enum.ToObject(typeof(CDS_NODE_CMD_Enum), Convert.ToInt32(e.Value))).ToString();
                        break;
                    }

                case CDS_NODE_CMD_Enum.CDS_PTS_TMP_GET:
                    {
                        responders[e.Address].Temperature.Value = e.Value;
                        break;
                    }
                case CDS_NODE_CMD_Enum.CDS_PTS_PRS_GET:
                    {
                        responders[e.Address].Pressure.Value = e.Value;
                        break;
                    }
                case CDS_NODE_CMD_Enum.CDS_CRE_TMP_GET:
                    {
                        responders[e.Address].CoreTemperature.Value = e.Value;
                        break;
                    }
                case CDS_NODE_CMD_Enum.CDS_SLP_GET:
                    {
                        responders[e.Address].SleepMode.Value = (SLP_MODE_Enum)(Convert.ToInt32(e.Value));
                        break;
                    }
                case CDS_NODE_CMD_Enum.CDS_STY_GET:
                    {
                        responders[e.Address].Salinity.Value = e.Value;
                        break;
                    }

                case CDS_NODE_CMD_Enum.CDS_CMD_RSV_0:
                case CDS_NODE_CMD_Enum.CDS_CMD_RSV_1:
                case CDS_NODE_CMD_Enum.CDS_CMD_RSV_2:
                case CDS_NODE_CMD_Enum.CDS_CMD_RSV_3:
                case CDS_NODE_CMD_Enum.CDS_CMD_RSV_4:
                case CDS_NODE_CMD_Enum.CDS_CMD_RSV_5:
                case CDS_NODE_CMD_Enum.CDS_CMD_ZDPT_ADJ:

                case CDS_NODE_CMD_Enum.CDS_USR_CMD_0:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_1:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_2:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_3:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_4:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_5:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_6:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_7:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_8:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_9:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_10:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_11:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_12:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_13:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_14:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_15:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_16:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_17:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_18:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_19:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_20:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_21:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_22:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_23:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_24:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_25:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_26:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_27:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_28:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_29:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_30:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_31:
                case CDS_NODE_CMD_Enum.CDS_USR_CMD_32:

                case CDS_NODE_CMD_Enum.CDS_RESERVED_0:
                case CDS_NODE_CMD_Enum.CDS_RESERVED_1:
                case CDS_NODE_CMD_Enum.CDS_RESERVED_2:
                case CDS_NODE_CMD_Enum.CDS_RESERVED_3:
                case CDS_NODE_CMD_Enum.CDS_RESERVED_4:
                case CDS_NODE_CMD_Enum.CDS_RESERVED_5:
                case CDS_NODE_CMD_Enum.CDS_RESERVED_6:
                    {
                        responders[e.Address].LastQueryResult.Value = e.CommandID.ToString();
                        break;
                    }

                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_01:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_02:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_03:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_04:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_05:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_06:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_07:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_08:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_09:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_10:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_11:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_12:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_13:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_14:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_15:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_16:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_17:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_18:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_19:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_20:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_21:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_22:
                case CDS_NODE_CMD_Enum.CDS_SET_ADDR_23:
                    {
                        responders[e.Address].LastQueryResult.Value = e.CommandID.ToString();
                        break;
                    }

                case CDS_NODE_CMD_Enum.CDS_ERR_NSUPP:
                case CDS_NODE_CMD_Enum.CDS_ERR_NAVAIL:
                case CDS_NODE_CMD_Enum.CDS_ERR_RES_0:
                case CDS_NODE_CMD_Enum.CDS_ERR_RES_1:
                case CDS_NODE_CMD_Enum.CDS_ERR_RES_2:
                case CDS_NODE_CMD_Enum.CDS_ERR_RES_3:
                case CDS_NODE_CMD_Enum.CDS_ERR_RES_4:
                case CDS_NODE_CMD_Enum.CDS_ERR_RES_5:
                case CDS_NODE_CMD_Enum.CDS_ERR_RES_6:
                case CDS_NODE_CMD_Enum.CDS_ERR_BAT_LOW:
                    {
                        /// TODO: warning !!!
                        responders[e.Address].LastQueryResult.Value = e.CommandID.ToString();
                        break;
                    }
            }


            ProcessResponder(e.Address, e.Azimuth);
            ZPortState = PortState.OK;
            RespondersUpdatedEventHandler.Rise(this, new EventArgs());
            OnActionResult("OK");
        }

        private void zport_RemoteTimeout(object sender, RemoteTimeoutEventArgs e)
        {
            if (!responders.ContainsKey(e.Address))
                responders.Add(e.Address, new ZResponder(e.Address));

            responders[e.Address].IsTimeout.Value = true;

            RespondersUpdatedEventHandler.Rise(this, new EventArgs());
            ZPortState = PortState.OK;

            OnActionResult("TIMEOUT");
        }

        private void zport_SystemOrientationUpdated(object sender, EventArgs e)
        {
            Pitch.Value = zport.StationPitch;
            Roll.Value = zport.StationRoll;

            StationUpdatedEventHandler.Rise(this, new EventArgs());
            ZPortState = PortState.OK;
        }

        private void zport_SystemStateUpdated(object sender, EventArgs e)
        {
            Depth.Value = zport.StationDepth + stationDepthAdjust;
            Temperature.Value = zport.StationTemperature;

            if ((zport.TRX_State == TRXState_Enum.READY) && (isAutoQuery) && (!zport.IsBusy))
            {
                ProcessRemoteQueries();
            }

            if (zport.StationDepth < ZMA.StationMinimalAllowedDepth)
            {
                isDepthLessThanAllowed = true;
                LogEventHandler.Rise(this,
                    new LogEventArgs(LogLineType.ERROR,
                        string.Format("Station depth ({0}) is less than allowed ({1})", zport.StationDepth, ZMA.StationMinimalAllowedDepth)));
            }
            else
            {
                isDepthLessThanAllowed = false;
            }

            ZPortState = PortState.OK;
            StationUpdatedEventHandler.Rise(this, new EventArgs());
        }

        private void port_StationACKReceived(object sender, StationACKEventArgs e)
        {            
            if ((e.QueryID == ICs.IC_D2H_REM_REQ_EX) && (e.Result == LocalError_Enum.LOC_ERR_UNSUPPORTED))
            {
                isStationRemoteQueryExSupported = false;
            }
            
            ZPortState = PortState.OK;
        }

        #endregion

        #region nmeaListener

        private void nmeaListener_HDGReceived(object sender, HDGMessageEventArgs e)
        {
            if (!isHDTPresent)
            {
                if (e.IsValid)
                {
                    if (IsUseVTGAsHeadingSource)
                        IsUseVTGAsHeadingSource = false;

                    Azimuth.Value = e.MagneticHeading;
                    StationUpdatedEventHandler.Rise(this, new EventArgs());
                    HDGPortState = PortState.OK;                    
                }
                else
                {
                    HDGPortState = PortState.WAITING;
                }
            }
        }

        private void nmeaListener_HDTReceived(object sender, HDTMessageEventArgs e)
        {
            if (e.IsValid)
            {
                if (!isHDTPresent)
                    isHDTPresent = true;

                if (IsUseVTGAsHeadingSource)
                    IsUseVTGAsHeadingSource = false;

                Azimuth.Value = e.Heading;
                StationUpdatedEventHandler.Rise(this, new EventArgs());

                HDGPortState = PortState.OK;
            }
            else
            {
                HDGPortState = PortState.WAITING;
            }
        }

        private void nmeaListener_VTGReceived(object sender, VTGMessageEventArgs e)
        {
            if (e.IsValid)
            {
                SpeedKmh.Value = e.SpeedKmh;
                VTGTrack.Value = e.TrackTrue;

                if (IsUseVTGAsHeadingSource)
                    Azimuth.Value = e.TrackTrue;
            }
        }

        private void nmeaListener_RMCReceived(object sender, RMCMessageEventArgs e)
        {
            if (e.IsValid)
            {
                Latitude.Value = e.Latitude;
                Longitude.Value = e.Longitude;
                GNSSTime.Value = e.TimeFix;
                GNSSPortState = PortState.OK;

                if (Depth.IsInitializedAndNotObsolete)
                {
                    GeoLocationUpdatedEventHandler.Rise(this,
                        new GeoLocationUpdateEventArgs("Station", 
                            DateTime.Now, e.Latitude, e.Longitude, Depth.Value));
                }
            }
            else
            {
                GNSSPortState = PortState.WAITING;
            }
        }

        #endregion

        #endregion

        #region Events

        public EventHandler<LogEventArgs> LogEventHandler;

        public EventHandler ZPortStateChangedEventHandler;
        public EventHandler GNSSPortStateChangedEventHandler;
        public EventHandler HDGPortStateChangedEventHandler;
        public EventHandler OutPortStateChangedEventHandler;

        public EventHandler<LocDataUpdatedEventArgs> LocalDataUpdatedEventHandler;

        public EventHandler RespondersUpdatedEventHandler;
        public EventHandler StationUpdatedEventHandler;
        public EventHandler IsStationDeviceInfoUpdatedChangedEventHandler;

        public EventHandler<StringEventArgs> RemoteActionProgessEventHandler;
        public EventHandler<StringEventArgs> WrittenToOutputPortEventHandler;

        public EventHandler On1PPSEventHandler;

        public EventHandler<GeoLocationUpdateEventArgs> GeoLocationUpdatedEventHandler;

        #endregion

        #region IDisposable

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {
            if (!this.disposed)
            {
                if (disposing)
                {
                    if (timer.IsRunning)
                        timer.Stop();
                    timer.Dispose();

                    if (zport.IsOpen)
                    {
                        try
                        {
                            zport.Close();
                        }
                        catch { }
                    }

                    zport.Dispose();

                    if (isAUXPortsUsed)
                    {
                        try
                        {
                            auxPorts.Close();
                            auxPorts.Dispose();
                        }
                        catch { }
                    }

                    if (isOutPortUsed)
                    {
                        if (outPort.IsOpen)
                        {
                            try
                            {
                                outPort.Close();
                            }
                            catch { }
                        }

                        outPort.Dispose();
                    }
                }

                disposed = true;
            }
        }

        #endregion
    }
}
