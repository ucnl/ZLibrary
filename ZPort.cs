using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Reflection;
using UCNLDrivers;
using UCNLNMEA;
using ZLibrary.Localization;

namespace ZLibrary
{
    #region Custom EventArgs

    public class RemoteTimeoutEventArgs : EventArgs
    {
        #region Properties

        public ZAddress Address { get; private set; }
        public CDS_NODE_CMD_Enum CommandID { get; private set; }

        #endregion

        #region Constructor

        public RemoteTimeoutEventArgs(ZAddress id, CDS_NODE_CMD_Enum cmdID)
        {
            Address = id;
            CommandID = cmdID;
        }

        #endregion
    }

    public class RemoteResponseEventArgs : EventArgs
    {
        #region Properties

        public ZAddress Address { get; private set; }
        public CDS_NODE_CMD_Enum CommandID { get; private set; }
        public double Value { get; private set; }

        public double Azimuth { get; private set; }
        public double Distance { get; private set; }
        public double SNR { get; private set; }
        public double Dpl { get; private set; }

        public int DFlag { get; private set; }

        #endregion

        #region Constructor

        public RemoteResponseEventArgs(ZAddress id, CDS_NODE_CMD_Enum cmdID, double value, double azm, double dst, double snr, double dpl, int dFlag)
        {
            Address = id;
            CommandID = cmdID;
            Value = value;
            Azimuth = azm;
            Distance = dst;
            SNR = snr;
            Dpl = dpl;
            DFlag = dFlag;
        }

        #endregion
    }

    public class LocDataUpdatedEventArgs : EventArgs
    {
        #region Properties

        public LOC_DATA_ID DataID { get; private set; }
        public double DataValue { get; private set; }

        #endregion

        #region Constructor

        public LocDataUpdatedEventArgs(LOC_DATA_ID dataID, double dataVal)
        {
            DataID = dataID;
            DataValue = dataVal;
        }

        #endregion
    }

    public class StationACKEventArgs : EventArgs
    {
        #region Properties

        public ICs QueryID { get; private set; }
        public LocalError_Enum Result { get; private set; }

        #endregion

        #region Constructor

        public StationACKEventArgs(ICs queryID, LocalError_Enum result)
        {
            QueryID = queryID;
            Result = result;
        }

        #endregion
    }

    #endregion

    public class ZPort : IDisposable
    {
        #region Properties

        NMEASerialPort port;
        PrecisionTimer timer;

        bool disposed = false;

        public bool IsOpen
        {
            get { return port.IsOpen; }
        }


        public bool IsDeviceInfoUpdated { get; private set; }

        public string SystemMoniker { get; private set; }
        public string SystemVersion { get; private set; }
        public string CoreMoniker { get; private set; }
        public string CoreVersion { get; private set; }
        public string Serial { get; private set; }

        public double StationPitch { get; private set; }
        public double StationRoll { get; private set; }

        public AHRSMode_Enum Ahrs_Mode { get; private set; }
        public double StationTemperature { get; private set; }
        public double StationDepth { get; private set; }
        public TRXState_Enum TRX_State { get; private set; }

        public string PortName
        {
            get { return port.PortName; }
        }

        bool isBusy = false;
        public bool IsBusy
        {
            get { return isBusy; }
            private set
            {
                isBusy = value;
                IsBusyChangedEventHandler.Rise(this, new EventArgs());

                if (value)
                    query_tmoCnt = 0;
            }
        }

        string queriedStr = string.Empty;
        ICs queryID = ICs.IC_INVALID;

        private delegate void parserDelegate(object[] parameters);
        private Dictionary<ICs, parserDelegate> parsers;


        int query_tmoCnt = 0;
        int sys_state_tmoCnt = 0;

        int max_sys_state_tmo;
        int max_query_tmo;

        static bool nmeaSingleton = false;

        #endregion

        #region Constructor

        public ZPort(SerialPortSettings portSettings)
        {
            LocStringManager.Init("ZLibrary.Localization.LocStringResources", Assembly.GetExecutingAssembly());

            port = new NMEASerialPort(portSettings);

            port.PortError += (o, e) => PortErrorEventHandler.Rise(o, e);
            port.NewNMEAMessage += new EventHandler<NewNMEAMessageEventArgs>(port_NewNMEAMessage);

            timer = new PrecisionTimer();
            timer.Period = 100;
            timer.Mode = Mode.Periodic;
            timer.Tick += new EventHandler(timer_Tick);

            max_query_tmo = ZMA.MaxQueryTimeout_s * 1000 / timer.Period;
            max_sys_state_tmo = 5 + ZMA.SystemStateUpdatePeriod_s * 1000 / timer.Period;

            TRX_State = TRXState_Enum.UNKNOWN;

            parsers = new Dictionary<ICs, parserDelegate>()
            {
                { ICs.IC_D2H_DEVICE_INFO, new parserDelegate(DEVICE_INFO_Parse) },
                { ICs.IC_D2H_ACK, new parserDelegate(ACK_Parse) },
                { ICs.IC_D2H_FLD_VAL, new parserDelegate(FLD_VAL_Parse) },                
                { ICs.IC_D2H_LOC_DATA_VAL, new parserDelegate(LOC_DATA_Parse) },                
                { ICs.IC_D2H_BASE_REQ, new parserDelegate(BASE_REQ_Parse) },                
                { ICs.IC_D2H_REM_TOUT, new parserDelegate(REM_TOUT_Parse) },
                { ICs.IC_D2H_REM_RESP, new parserDelegate(REM_RESP_Parse) },
                { ICs.IC_D2H_SYS_STATE, new parserDelegate(SYS_STATE_Parse) },
                { ICs.IC_D2H_INC_DATA, new parserDelegate(INC_DATA_Parse) }
            };

            if (!nmeaSingleton)
            {
                nmeaSingleton = true;

                #region NMEA ZMA init

                NMEAParser.AddManufacturerToProprietarySentencesBase(ManufacturerCodes.ZMA);

                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "!", "c--c,x,c--c,x,x,c--c");      // IC_D2H_DEVICE_INFO
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "0", "x");                         // IC_D2H_ACK
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "1", "x,x");                       // IC_H2D_FLD_GET
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "2", "x,x");                       // IC_H2D_FLD_SET
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "3", "x,x");                       // IC_D2H_FLD_VAL
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "4", "x,x");                       // IC_H2D_LOC_DATA_GET
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "5", "x,x.x");                     // IC_H2D_LOC_DATA_SET
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "6", "x,x.x");                     // IC_D2H_LOC_DATA_VAL
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "7", "x,x.x");                     // IC_H2D_LOC_INVOKE
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "8", "x");                         // IC_H2D_RPH_MODE_SET

                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "9", "x.x,x.x,x.x,x");             // IC_H2D_WP_SET

                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "A", "x.x,x.x,x.x,x.x");           // IC_D2H_LD
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "B", "x,x.x,x.x");                 // IC_D2H_BASE_REQ
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "C", "x,x");                       // IC_D2H_REM_REQ
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "D", "x,x");                       // IC_D2H_REM_TOUT
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "E", "x,x,x,x.x,x.x,x.x,x.x,x.x"); // IC_D2H_REM_RESP
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "F", "x.x,x.x,x,x");               // IC_D2H_SYS_STATE
                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "G", "x.x,x.x");                   // IC_D2H_INC_DATA

                NMEAParser.AddProprietarySentenceDescription(ManufacturerCodes.ZMA, "H", "x,x,x");                     // IC_D2H_REM_REQ_EX

                #endregion
            }
        }

        #endregion

        #region Methods

        #region Public

        public void Open()
        {
            if (port.IsOpen)
                throw new InvalidOperationException(string.Format("{0} {1} (ZMA) {2}", 
                    LocStringManager.Port_str,
                    port.PortName,
                    LocStringManager.IsAlreadyOpened_str));
            else
            {
                port.Open();
                timer.Start();

                IsDeviceInfoUpdated = false;

                SystemMoniker = string.Empty;
                SystemVersion = string.Empty;
                CoreMoniker = string.Empty;
                CoreVersion = string.Empty;
                Serial = string.Empty;

                StationDepth = double.NaN;
                StationTemperature = double.NaN;
                TRX_State = TRXState_Enum.UNKNOWN;

                StationPitch = double.NaN;
                StationRoll = double.NaN;

                IsBusy = false;
            }
        }

        public void Close()
        {
            if (timer.IsRunning)
                timer.Stop();

            if (port.IsOpen)
                port.Close();
        }

        public void DataReceived_Emul(string data)
        {
            port_NewNMEAMessage(port, new NewNMEAMessageEventArgs(data));
        }

        public void QueryLocalDataGet(LOC_DATA_ID dataID)
        {
            var msg = NMEAParser.BuildProprietarySentence(ManufacturerCodes.ZMA, "4", new object[] { dataID, 0 });
            TrySend(msg, ICs.IC_H2D_LOC_DATA_GET);
        }

        public void QueryLocalDataSet(LOC_DATA_ID dataID, double value)
        {
            var msg = NMEAParser.BuildProprietarySentence(ManufacturerCodes.ZMA, "5", new object[] { dataID, value });
            TrySend(msg, ICs.IC_H2D_LOC_DATA_SET);
        }

        public void QueryLocInvoke(LOC_INVOKE_ID actionID, double prm)
        {
            var msg = NMEAParser.BuildProprietarySentence(ManufacturerCodes.ZMA, "7", new object[] { actionID, prm });
            TrySend(msg, ICs.IC_H2D_LOC_INVOKE);
        }

        public void QueryLocRPhoneModeSet(bool isRedPhoneMode)
        {
            int isMode = 0;
            if (isRedPhoneMode) isMode = 1;

            var msg = NMEAParser.BuildProprietarySentence(ManufacturerCodes.ZMA, "8", new object[] { isMode });
            TrySend(msg, ICs.IC_H2D_RPH_MODE_SET);
        }

        public void QueryRemote(ZAddress address, CDS_NODE_CMD_Enum cmd)
        {
            if (address != ZAddress.INVALID)
            {
                var msg = NMEAParser.BuildProprietarySentence(ManufacturerCodes.ZMA, "C", new object[] { (int)address, cmd });
                TrySend(msg, ICs.IC_H2D_REM_REQ);
            }
            else
                throw new ArgumentOutOfRangeException("address");
        }               

        public void QueryRemoteEx(ZAddress address, CDS_NODE_CMD_Enum cmd, double rev_azimuth_to_station)
        {
            if (address != ZAddress.INVALID)
            {
                var msg = NMEAParser.BuildProprietarySentence(ManufacturerCodes.ZMA, "H", new object[] { (int)address, cmd, Convert.ToInt32(rev_azimuth_to_station) });
                TrySend(msg, ICs.IC_H2D_REM_REQ);
            }
            else
                throw new ArgumentOutOfRangeException("address");
        }

        #endregion

        #region Private

        #region parsers

        private void ACK_Parse(object[] parameters)
        {
            LocalError_Enum errCode = LocalError_Enum.LOC_ERR_INVALID;

            try
            {
                errCode = (LocalError_Enum)(int)parameters[0];
            }
            catch (Exception ex)
            {
                LogEventHandler.Rise(this, new LogEventArgs(LogLineType.ERROR, ex));
            }

            if (errCode != LocalError_Enum.LOC_ERR_INVALID)
            {
                LogEventHandler.Rise(this, new LogEventArgs(LogLineType.INFO, string.Format("{0} ({1}) caused {2}", queryID, queriedStr, errCode)));
                StationACKEventHandler.Rise(this, new StationACKEventArgs(queryID, errCode));
                IsBusy = false;
            }
        }

        private void SYS_STATE_Parse(object[] parameters)
        {
            double temp = double.NaN;
            double depth = double.NaN;
            AHRSMode_Enum ahrsMode = AHRSMode_Enum.UNKNOWN;
            TRXState_Enum trxState = TRXState_Enum.UNKNOWN;
            bool isOK = false;

            try
            {
                temp = (double)parameters[0];
                depth = (double)parameters[1];
                ahrsMode = (AHRSMode_Enum)(int)parameters[2];
                trxState = (TRXState_Enum)(int)parameters[3];
                sys_state_tmoCnt = 0;
                isOK = true;
            }
            catch (Exception ex)
            {
                LogEventHandler.Rise(this, new LogEventArgs(LogLineType.ERROR, ex));
            }

            if (isOK)
            {
                sys_state_tmoCnt = 0;
                StationTemperature = temp;
                StationDepth = depth;
                Ahrs_Mode = ahrsMode;
                TRX_State = trxState;
                SystemStateUpdatedEventHandler.Rise(this, new EventArgs());
            }
        }

        private void REM_TOUT_Parse(object[] parameters)
        {
            ZAddress bAddress = ZAddress.INVALID;
            CDS_NODE_CMD_Enum nodeCMD = CDS_NODE_CMD_Enum.CDS_INVALID;

            try
            {
                bAddress = (ZAddress)(int)parameters[0];
                nodeCMD = (CDS_NODE_CMD_Enum)(int)parameters[1];
            }
            catch (Exception ex)
            {
                LogEventHandler.Rise(this, new LogEventArgs(LogLineType.ERROR, ex));
            }

            if ((bAddress != ZAddress.INVALID) && (nodeCMD != CDS_NODE_CMD_Enum.CDS_INVALID))
            {
                RemoteTimeoutEventHandler.Rise(this, new RemoteTimeoutEventArgs(bAddress, nodeCMD));
            }
        }

        private void REM_RESP_Parse(object[] parameters)
        {
            ZAddress address = ZAddress.INVALID;
            CDS_NODE_CMD_Enum nodeCMD = CDS_NODE_CMD_Enum.CDS_INVALID;
            int dFlag = -1;
            double azimuth = double.NaN;
            double distance = double.NaN;
            double dataValue = double.NaN;
            double snr = double.NaN;
            double dpl = double.NaN;
            bool isOk = false;

            try
            {
                address = (ZAddress)(int)parameters[0];
                nodeCMD = (CDS_NODE_CMD_Enum)(int)parameters[1];
                dFlag = (int)parameters[2];
                azimuth = (double)parameters[3];
                if (parameters[4] != null)
                    distance = (double)parameters[4];
                dataValue = (double)parameters[5];
                snr = (double)parameters[6];
                dpl = (double)parameters[7];

                isOk = true;
            }
            catch (Exception ex)
            {
                LogEventHandler.Rise(this, new LogEventArgs(LogLineType.ERROR, ex));
            }


            if (isOk)
            {
                RemoteResponseEventHandler.Rise(this, new RemoteResponseEventArgs(address, nodeCMD, dataValue, azimuth, distance, snr, dpl, dFlag));
            }

        }

        private void LOC_DATA_Parse(object[] parameters)
        {
            LOC_DATA_ID dataID = LOC_DATA_ID.LOC_DATA_UNKNOWN;
            double dataVal = double.NaN;

            try
            {
                dataID = (LOC_DATA_ID)(int)parameters[0];
                dataVal = (double)parameters[1];
            }
            catch (Exception ex)
            {
                LogEventHandler.Rise(this, new LogEventArgs(LogLineType.ERROR, ex));
            }

            if ((dataID != LOC_DATA_ID.LOC_DATA_UNKNOWN) && (!double.IsNaN(dataVal)))
            {
                LocalDataReceivedEventHandler.Rise(this, new LocDataUpdatedEventArgs(dataID, dataVal));
            }
        }

        private void INC_DATA_Parse(object[] parameters)
        {
            double roll = double.NaN;
            double pitch = double.NaN;            

            try
            {
                roll = (double)parameters[0];
                pitch = (double)parameters[1];                
            }
            catch (Exception ex)
            {
                LogEventHandler.Rise(this, new LogEventArgs(LogLineType.ERROR, ex));
            }

            if (!double.IsNaN(pitch) && !double.IsNaN(roll))
            {
                StationPitch = pitch;
                StationRoll = roll;
                SystemOrientationUpdatedEventHandler.Rise(this, new EventArgs());
            }
        }

        private void FLD_VAL_Parse(object[] parameters)
        {
            throw new NotImplementedException();
        }

        private void DEVICE_INFO_Parse(object[] parameters)
        {
            string sys_moniker = string.Empty;
            string sys_version = string.Empty;
            string core_moniker = string.Empty;
            string core_version = string.Empty;
            DEVICE_TYPE dev_type = DEVICE_TYPE.DEV_INVALID;
            string serial = string.Empty;

            bool isOk = false;

            try
            {
                sys_moniker = (string)parameters[0];
                sys_version = ZMA.BCDVer2Str((int)parameters[1]);
                core_moniker = (string)parameters[2];
                core_version = ZMA.BCDVer2Str((int)parameters[3]);
                dev_type = (DEVICE_TYPE)(int)parameters[4];
                serial = (string)parameters[5];
                isOk = true;
            }
            catch (Exception ex)
            {
                LogEventHandler.Rise(this, new LogEventArgs(LogLineType.ERROR, ex));
            }

            if (isOk)
            {
                SystemMoniker = sys_moniker;
                SystemVersion = sys_version;
                CoreMoniker = core_moniker;
                CoreVersion = core_version;
                Serial = serial;

                IsDeviceInfoUpdated = true;
                DeviceInfoUpdatedEventHandler.Rise(this, new EventArgs());
            }
        }

        private void BASE_REQ_Parse(object[] parameters)
        {
            throw new NotImplementedException();
        }

        #endregion

        private void TrySend(string msg, ICs qID)
        {
            if (!isBusy)
            {
                try
                {
                    port.SendData(msg);
                    IsBusy = true;
                    queriedStr = msg;
                    queryID = qID;
                    timer.Start();
                    LogEventHandler.Rise(this, new LogEventArgs(LogLineType.INFO, string.Format("{0} (ZMA) << {1}", port.PortName, msg)));
                }
                catch (Exception ex)
                {
                    LogEventHandler.Rise(this, new LogEventArgs(LogLineType.ERROR, ex));
                }
            }
        }


        #endregion

        #endregion

        #region Handlers

        #region port

        private void port_NewNMEAMessage(object sender, NewNMEAMessageEventArgs e)
        {
            bool isParsed = false;
            NMEASentence result = null;

            LogEventHandler.Rise(this, new LogEventArgs(LogLineType.INFO, string.Format("{0} (ZMA) >> {1}", PortName, e.Message)));

            try
            {
                result = NMEAParser.Parse(e.Message);
                isParsed = true;
            }
            catch (Exception ex)
            {
                LogEventHandler.Rise(this, new LogEventArgs(LogLineType.ERROR, ex));
            }

            if (isParsed)
            {
                if (result is NMEAProprietarySentence)
                {
                    NMEAProprietarySentence pResult = (result as NMEAProprietarySentence);

                    if (pResult.Manufacturer == ManufacturerCodes.ZMA)
                    {
                        ICs sentenceID = ZMA.ICsByMessageID(pResult.SentenceIDString);

                        if (sentenceID != ICs.IC_INVALID)
                        {
                            if (parsers.ContainsKey(sentenceID))
                                parsers[sentenceID](pResult.parameters);
                            else
                            {
                                // skip unsupported sentence
                                LogEventHandler.Rise(this, new LogEventArgs(LogLineType.INFO,
                                    string.Format("WARNING: unsupported sentence identifier \"{0}\" (\"{1}\") in \"{2}\"", sentenceID, pResult.SentenceIDString, e.Message)));
                            }
                        }
                        else
                        {
                            // skip unknown sentence ID
                            LogEventHandler.Rise(this, new LogEventArgs(LogLineType.INFO,
                                string.Format("WARNING: unsupported sentence identifier \"{0}\" in \"{1}\"", pResult.SentenceIDString, e.Message)));
                        }
                    }
                    else
                    {
                        // skip unsupported manufacturer ID
                        LogEventHandler.Rise(this, new LogEventArgs(LogLineType.INFO,
                            string.Format("WARNING: unsupported manufacturer identifier \"{0}\" in \"{1}\"", pResult.SentenceIDString, e.Message)));
                    }
                }
                else
                {
                    LogEventHandler.Rise(this, new LogEventArgs(LogLineType.INFO,
                        string.Format("WARNING: unsupported standard sentence \"{0}\"", e.Message)));
                }
            }

        }

        #endregion

        #region timer

        private void timer_Tick(object sender, EventArgs e)
        {
            if (++sys_state_tmoCnt >= max_sys_state_tmo)
            {
                sys_state_tmoCnt = 0;
                SystemTimeoutEventHandler.Rise(this, new EventArgs());
            }

            if (isBusy)
            {
                if (++query_tmoCnt > max_query_tmo)
                {
                    query_tmoCnt = 0;
                    IsBusy = false;
                }
            }
        }

        #endregion

        #endregion

        #region Events

        public EventHandler SystemTimeoutEventHandler;

        public EventHandler IsBusyChangedEventHandler;
        public EventHandler SystemStateUpdatedEventHandler;
        public EventHandler DeviceInfoUpdatedEventHandler;
        public EventHandler<RemoteTimeoutEventArgs> RemoteTimeoutEventHandler;
        public EventHandler<RemoteResponseEventArgs> RemoteResponseEventHandler;
        public EventHandler<LocDataUpdatedEventArgs> LocalDataReceivedEventHandler;
        public EventHandler SystemOrientationUpdatedEventHandler;
        public EventHandler<StationACKEventArgs> StationACKEventHandler;

        public EventHandler<SerialErrorReceivedEventArgs> PortErrorEventHandler;

        public EventHandler<LogEventArgs> LogEventHandler;

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

                    if (IsOpen)
                        Close();

                    port.Dispose();
                    timer.Dispose();
                }

                disposed = true;
            }
        }

        #endregion
    }
}
