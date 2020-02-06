package frc.robot.constants.wiring;

public enum CANWiring{

    DRIVE_LEFT_1 (25),
    DRIVE_LEFT_2 (24),
    DRIVE_LEFT_3 (23),
    DRIVE_RIGHT_1 (20),
    DRIVE_RIGHT_2 (21),
    DRIVE_RIGHT_3 (22);

        /** The stored port */
        private int m_port;

        /**
         * Initializes the constant for the port
         * @param port
         */
        private CANWiring (int port) {
            this.m_port = port;
        }
    
        /**
         * Returns the port for the specific constant
         * @return
         */
        public int getPort (){
            return this.m_port;
        }
}