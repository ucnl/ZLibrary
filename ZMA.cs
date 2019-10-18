using System;
using System.Collections.Generic;
using System.Text;
using System.Threading.Tasks;

namespace ZLibrary
{
    [Flags]
    public enum StationDataID : int
    {
        None = 0,
        TRX = 1,
        DPT = 2,
        TMP = 4,
        ROL = 8,
        PTC = 16,
        LAT = 32,
        LON = 64,
        AZM = 128,
        SPD = 256,
        VTG = 512
    }

    public enum ResponderDataID : int
    {
        DST = 0,
        AZM = 1,
        DPT = 2,
        BAT = 3,
        PRS = 4,
        TMP = 5,
        CTM = 6,
        TMO = 7,
        LQR = 8,
        LAT = 9,
        LON = 10,
        MSR = 11,
        DPL = 12,
        STY = 13,
        SLP = 14,
        DSTP = 15,
        INVALID
    }

    public enum AHRSMode_Enum : int
    {
        OFF = 0,
        ON = 1,
        EXT = 2,
        UNKNOWN
    }

    public enum TRXState_Enum : int
    {
        READY = 0,
        TX = 1,
        RX = 2,
        UNKNOWN
    }

    public enum SLP_MODE_Enum : int
    {
        SLP_MODE_1 = 0,
        SLP_MODE_2 = 1,
        SLP_MODE_4 = 2,
        SLP_MODE_8 = 3,
        SLP_MODE_10 = 4,
        SLP_MODE_20 = 5,
        SLP_MODE_30 = 6,
        SLP_MODE_40 = 7,
        SLP_MODE_50 = 8,
        SLP_MODE_NEVER = 9,
        SLP_MODE_INVALID
    }

    public enum ZAddress : int
    {
        Responder_1 = 1,
        Responder_2 = 2,
        Responder_3 = 3,
        Responder_4 = 4,
        Responder_5 = 5,
        Responder_6 = 6,
        Responder_7 = 7,
        Responder_8 = 8,
        Responder_9 = 9,
        Responder_10 = 10,
        Responder_11 = 11,
        Responder_12 = 12,
        Responder_13 = 13,
        Responder_14 = 14,
        Responder_15 = 15,
        Responder_16 = 16,
        Responder_17 = 17,
        Responder_18 = 18,
        Responder_19 = 19,
        Responder_20 = 20,
        Responder_21 = 21,
        Responder_22 = 22,
        Responder_23 = 23,
        INVALID
    }

    public enum CDS_NODE_CMD_Enum : int
    {
        CDS_PING = 361,
        CDS_DPT_GET = 362,
        CDS_STY_SET_0 = 363,
        CDS_STY_SET_1 = 364,
        CDS_STY_SET_2 = 365,
        CDS_STY_SET_3 = 366,
        CDS_STY_SET_4 = 367,
        CDS_STY_SET_5 = 368,
        CDS_STY_SET_6 = 369,
        CDS_STY_SET_7 = 370,
        CDS_STY_SET_8 = 371,
        CDS_STY_SET_9 = 372,
        CDS_STY_SET_10 = 373,
        CDS_STY_SET_11 = 374,
        CDS_STY_SET_12 = 375,
        CDS_STY_SET_13 = 376,
        CDS_STY_SET_14 = 377,
        CDS_STY_SET_15 = 378,
        CDS_STY_SET_16 = 379,
        CDS_STY_SET_17 = 380,
        CDS_STY_SET_18 = 381,
        CDS_STY_SET_19 = 382,
        CDS_STY_SET_20 = 383,
        CDS_STY_SET_21 = 384,
        CDS_STY_SET_22 = 385,
        CDS_STY_SET_23 = 386,
        CDS_STY_SET_24 = 387,
        CDS_STY_SET_25 = 388,
        CDS_STY_SET_26 = 389,
        CDS_STY_SET_27 = 390,
        CDS_STY_SET_28 = 391,
        CDS_STY_SET_29 = 392,
        CDS_STY_SET_30 = 393,
        CDS_STY_SET_31 = 394,
        CDS_STY_SET_32 = 395,
        CDS_STY_SET_33 = 396,
        CDS_STY_SET_34 = 397,
        CDS_STY_SET_35 = 398,
        CDS_STY_SET_36 = 399,
        CDS_STY_SET_37 = 400,
        CDS_STY_SET_38 = 401,
        CDS_STY_SET_39 = 402,
        CDS_STY_SET_40 = 403,

        CDS_SLP_SET_59_60 = 404,
        CDS_SLP_SET_58_60 = 405,
        CDS_SLP_SET_56_60 = 406,
        CDS_SLP_SET_52_60 = 407,
        CDS_SLP_SET_50_60 = 408,
        CDS_SLP_SET_40_60 = 409,
        CDS_SLP_SET_30_60 = 410,
        CDS_SLP_SET_20_60 = 411,
        CDS_SLP_SET_10_60 = 412,
        CDS_SLP_SET_NEVER = 413,

        CDS_BAT_CHG_GET = 414,
        CDS_PTS_TMP_GET = 415,
        CDS_PTS_PRS_GET = 416,
        CDS_CRE_TMP_GET = 417,
        CDS_SLP_GET = 418,
        CDS_STY_GET = 419,

        CDS_CMD_RSV_0 = 420,
        CDS_CMD_RSV_1 = 421,
        CDS_CMD_RSV_2 = 422,
        CDS_CMD_RSV_3 = 423,
        CDS_CMD_RSV_4 = 424,
        CDS_CMD_RSV_5 = 425,
        CDS_CMD_ZDPT_ADJ = 426,

        CDS_USR_CMD_0 = 427,
        CDS_USR_CMD_1 = 428,
        CDS_USR_CMD_2 = 429,
        CDS_USR_CMD_3 = 430,
        CDS_USR_CMD_4 = 431,
        CDS_USR_CMD_5 = 432,
        CDS_USR_CMD_6 = 433,
        CDS_USR_CMD_7 = 434,
        CDS_USR_CMD_8 = 435,
        CDS_USR_CMD_9 = 436,
        CDS_USR_CMD_10 = 437,
        CDS_USR_CMD_11 = 438,
        CDS_USR_CMD_12 = 439,
        CDS_USR_CMD_13 = 440,
        CDS_USR_CMD_14 = 441,
        CDS_USR_CMD_15 = 442,
        CDS_USR_CMD_16 = 443,
        CDS_USR_CMD_17 = 444,
        CDS_USR_CMD_18 = 445,
        CDS_USR_CMD_19 = 446,
        CDS_USR_CMD_20 = 447,
        CDS_USR_CMD_21 = 448,
        CDS_USR_CMD_22 = 449,
        CDS_USR_CMD_23 = 450,
        CDS_USR_CMD_24 = 451,
        CDS_USR_CMD_25 = 452,
        CDS_USR_CMD_26 = 453,
        CDS_USR_CMD_27 = 454,
        CDS_USR_CMD_28 = 455,
        CDS_USR_CMD_29 = 456,
        CDS_USR_CMD_30 = 457,
        CDS_USR_CMD_31 = 458,
        CDS_USR_CMD_32 = 459,

        CDS_RESERVED_0 = 460,
        CDS_RESERVED_1 = 461,
        CDS_RESERVED_2 = 462,
        CDS_RESERVED_3 = 463,
        CDS_RESERVED_4 = 464,
        CDS_RESERVED_5 = 465,
        CDS_RESERVED_6 = 466,
        CDS_DPT_XM_GET = 467,

        CDS_SET_ADDR_01 = 468,
        CDS_SET_ADDR_02 = 469,
        CDS_SET_ADDR_03 = 470,
        CDS_SET_ADDR_04 = 471,
        CDS_SET_ADDR_05 = 472,
        CDS_SET_ADDR_06 = 473,
        CDS_SET_ADDR_07 = 474,
        CDS_SET_ADDR_08 = 475,
        CDS_SET_ADDR_09 = 476,
        CDS_SET_ADDR_10 = 477,
        CDS_SET_ADDR_11 = 478,
        CDS_SET_ADDR_12 = 479,
        CDS_SET_ADDR_13 = 480,
        CDS_SET_ADDR_14 = 481,
        CDS_SET_ADDR_15 = 482,
        CDS_SET_ADDR_16 = 483,
        CDS_SET_ADDR_17 = 484,
        CDS_SET_ADDR_18 = 485,
        CDS_SET_ADDR_19 = 486,
        CDS_SET_ADDR_20 = 487,
        CDS_SET_ADDR_21 = 488,
        CDS_SET_ADDR_22 = 489,
        CDS_SET_ADDR_23 = 490,
        CDS___________0 = 491, // !!!!!
        CDS___________1 = 492,
        CDS___________2 = 493,
        CDS___________3 = 494,
        CDS___________4 = 495,
        CDS___________5 = 496,
        CDS___________6 = 497,
        CDS___________7 = 498,
        CDS___________8 = 499,

        CDS_ERR_NSUPP = 500,
        CDS_ERR_NAVAIL = 501,
        CDS_ERR_RES_0 = 502,
        CDS_ERR_RES_1 = 503,
        CDS_ERR_RES_2 = 504,
        CDS_ERR_RES_3 = 505,
        CDS_ERR_RES_4 = 506,
        CDS_ERR_RES_5 = 507,
        CDS_ERR_RES_6 = 508,
        CDS_ERR_BAT_LOW = 509,

        CDS_INVALID
    }

    public enum LocalError_Enum : int
    {
        LOC_ERR_NO_ERROR = 0,
        LOC_ERR_INVALID_SYNTAX = 1,
        LOC_ERR_UNSUPPORTED = 2,
        LOC_ERR_TRANSMITTER_BUSY = 3,
        LOC_ERR_ARGUMENT_OUT_OF_RANGE = 4,
        LOC_ERR_INVALID_OPERATION = 5,
        LOC_ERR_UNKNOWN_FIELD_ID = 6,
        LOC_ERR_VALUE_UNAVAILIBLE = 7,
        LOC_ERR_RECEIVER_BUSY = 8,
        LOC_ERR_WAKE_UP = 9,
        LOC_ERR_STAND_BY = 10,

        LOC_ERR_INVALID
    }

    public enum LOC_DATA_ID : int
    {
        LOC_DATA_DEVICE_INFO = 0,
        LOC_DATA_MAX_REMOTE_TIMEOUT = 1,
        LOC_DATA_MAX_SUBSCRIBERS = 2,
        LOC_DATA_PTS_PRESSURE = 3,
        LOC_DATA_PTS_TEMPERATURE = 4,
        LOC_DATA_PTS_DEPTH = 5,
        LOC_DATA_CORE_TEMPERATURE = 6,
        LOC_DATA_BAT_CHARGE = 7,

        LOC_DATA_PRESSURE_RATING = 8,
        LOC_DATA_ZERO_PRESSURE = 9,
        LOC_DATA_WATER_DENSITY = 10,
        LOC_DATA_SALINITY = 11,
        LOC_DATA_SOUNDSPEED = 12,
        LOC_DATA_GRAVITY_ACC = 13,

        LOC_DATA_YEAR = 14,
        LOC_DATA_MONTH = 15,
        LOC_DATA_DATE = 16,
        LOC_DATA_HOUR = 17,
        LOC_DATA_MINUTE = 18,
        LOC_DATA_SECOND = 19,

        LOC_DATA_AHRS_MODE = 20,

        LOC_DATA_MAX_DIST = 21,

        LOC_DATA_UNKNOWN
    }

    public enum LOC_INVOKE_ID : int
    {
        LOC_INVOKE_FLASH_WRITE = 0,
        LOC_INVOKE_DPT_ZERO_ADJUST = 1,
        LOC_INVOKE_SYSTEM_RESET = 2,
        LOC_INVOKE_STAND_BY = 3,
        LOC_INVOKE_UNKNOWN
    }

    public enum DEVICE_TYPE : int
    {
        DEV_BASE = 0, // USBL base station
        DEV_NODE = 1, // Addressed responder
        DEV_INVALID
    }

    public enum ICs
    {
        IC_D2H_DEVICE_INFO,
        IC_D2H_ACK,
        IC_H2D_FLD_GET,
        IC_H2D_FLD_SET,
        IC_D2H_FLD_VAL,
        IC_H2D_LOC_DATA_GET,
        IC_H2D_LOC_DATA_SET,
        IC_D2H_LOC_DATA_VAL,
        IC_H2D_LOC_INVOKE,
        IC_H2D_RPH_MODE_SET,

        IC_H2D_WP_SET,
        IC_D2H_BASE_REQ,
        IC_H2D_REM_REQ,
        IC_D2H_REM_TOUT,
        IC_D2H_REM_RESP,
        IC_D2H_SYS_STATE,
        IC_D2H_INC_DATA,

        IC_D2H_REM_REQ_EX,

        IC_INVALID
    }


    public static class ZMA
    {
        public static readonly int CDS_MAX_SUBSCRIBERS = 23;

        public static readonly int CDS_MAX_REDPHONE_DIVERS = 4;

        public static readonly int CDS_BASE_LPID = 0;
        public static readonly int CDS_NODE_FIRST_LPID = CDS_BASE_LPID + 1;
        public static readonly int CDS_NODE_LAST_LPID = CDS_NODE_FIRST_LPID + CDS_MAX_SUBSCRIBERS - 1;

        public static readonly int CDS_MAX_CODES_PWR_9 = 511;
        public static readonly int CDS_ERROR_CODE = 512;
        public static readonly int CDS_MIN_U9_VAL = 0;
        public static readonly int CDS_MAX_U9_VAL = 499;

        public static readonly int Temp_Periodicity = 120;
        public static readonly int Bat_Periodicity = 240;

        public static readonly int MinSalinityPPM = 0;
        public static readonly int MaxSalinityPPM = 40;

        public static readonly int MinBatChargeV = 0;
        public static readonly int MaxBatChargeV = 20;

        public static readonly double StationMinimalAllowedDepth = 1.5;

        public static readonly int SystemStateUpdatePeriod_s = 1;
        public static readonly int MaxQueryTimeout_s = 4;

        public static readonly double MinMaxDistance_m = 500;
        public static readonly double MaxMaxDistance_m = 8000;

        public static void Rise(this EventHandler handler, object sender, EventArgs e)
        {
            if (handler != null)
                handler(sender, e);
        }

        public static void Rise<TEventArgs>(this EventHandler<TEventArgs> handler,
            object sender, TEventArgs e) where TEventArgs : EventArgs
        {
            if (handler != null)
                handler(sender, e);
        }

        public static bool IsInRangeInclusive(this double value, double minValue, double maxValue)
        {
            return ((value >= minValue) && (value <= maxValue));
        }

        public static string ToString(this ZAddress item)
        {
            if (item != ZAddress.INVALID)
                return item.ToString().Replace("_", " #");
            else
                return item.ToString();
        }

        public static bool CDS_IS_USR_CMD(CDS_NODE_CMD_Enum value)
        {
            return ((value >= CDS_NODE_CMD_Enum.CDS_USR_CMD_0) && (value <= CDS_NODE_CMD_Enum.CDS_USR_CMD_32));
        }

        public static CDS_NODE_CMD_Enum[] Select_USR_CMDs()
        {
            List<CDS_NODE_CMD_Enum> result = new List<CDS_NODE_CMD_Enum>();
            var vals = (CDS_NODE_CMD_Enum[])Enum.GetValues(typeof(CDS_NODE_CMD_Enum));

            foreach (var cmd in vals)
                if (CDS_IS_USR_CMD(cmd))
                    result.Add(cmd);

            return result.ToArray();
        }

        public static string[] Select_USR_CMDs_Names()
        {
            List<string> result = new List<string>();
            var vals = (CDS_NODE_CMD_Enum[])Enum.GetValues(typeof(CDS_NODE_CMD_Enum));

            foreach (var cmd in vals)
                if (CDS_IS_USR_CMD(cmd))
                    result.Add(cmd.ToString());

            return result.ToArray();
        }

        public static string BCDVer2Str(int versionData)
        {
            return string.Format("{0}.{1}", (versionData >> 0x08).ToString(), (versionData & 0xff).ToString("X2"));
        }

        static Dictionary<string, ICs> ICsIdxArray = new Dictionary<string, ICs>()
        {
            { "!", ICs.IC_D2H_DEVICE_INFO },
            { "0", ICs.IC_D2H_ACK },
            { "1", ICs.IC_H2D_FLD_GET },
            { "2", ICs.IC_H2D_FLD_SET },
            { "3", ICs.IC_D2H_FLD_VAL },
            { "4", ICs.IC_H2D_LOC_DATA_GET },
            { "5", ICs.IC_H2D_LOC_DATA_SET },
            { "6", ICs.IC_D2H_LOC_DATA_VAL },
            { "7", ICs.IC_H2D_LOC_INVOKE },
            { "8", ICs.IC_H2D_RPH_MODE_SET },
            { "9", ICs.IC_H2D_WP_SET },
            { "A", ICs.IC_D2H_BASE_REQ },
            { "B", ICs.IC_D2H_BASE_REQ },
            { "C", ICs.IC_H2D_REM_REQ },            
            { "D", ICs.IC_D2H_REM_TOUT },
            { "E", ICs.IC_D2H_REM_RESP },
            { "F", ICs.IC_D2H_SYS_STATE },
            { "G", ICs.IC_D2H_INC_DATA },
            { "H", ICs.IC_D2H_REM_REQ_EX },         
        };

        public static ICs ICsByMessageID(string msgID)
        {
            if (ICsIdxArray.ContainsKey(msgID))
                return ICsIdxArray[msgID];
            else
                return ICs.IC_INVALID;
        }
    }
}
