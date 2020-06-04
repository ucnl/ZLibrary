using System.Reflection;
using System.Resources;

namespace ZLibrary.Localization
{
    public static class LocStringManager
    {
        #region Properties

        static bool singleton = false;

        #region public

        public static string ZCoreShouldBeStartedFirst_str; // ZCore should be started first

        public static string UnableToProcessRequestInCurrentPortState_str; // Unable to procees request in current port state             
        public static string UnableToProcessRequestWhileIsAutoQueryModeIsSelected_str; // Unable to process request while IsAutoQuery mode is selected
        public static string OutputPortIsAlreadyInitialized_str; // Output port is already initialized

        public static string Port_str; //
        public static string IsAlreadyOpened_str;

        #endregion

        #endregion

        #region Methods

        public static void Init(string baseName, Assembly assembly)
        {
            if (!singleton)
            {
                ResourceManager locRM = new ResourceManager(baseName, assembly);

                ZCoreShouldBeStartedFirst_str = locRM.GetString("_ZCoreShouldBeStartedFirst");

                UnableToProcessRequestInCurrentPortState_str = locRM.GetString("_UnableToProcessRequestInCurrentPortState"); // Unable to procees request in current port state             
                UnableToProcessRequestWhileIsAutoQueryModeIsSelected_str = locRM.GetString("_UnableToProcessRequestWhileIsAutoQueryModeIsSelected"); // Unable to process request while IsAutoQuery mode is selected
                OutputPortIsAlreadyInitialized_str = locRM.GetString("_OutputPortIsAlreadyInitialized"); // Output port is already initialized
                Port_str = locRM.GetString("_Port");
                IsAlreadyOpened_str = locRM.GetString("_IsAlreadyOpened");

                singleton = true;
            }
        }

        #endregion
    }
}
