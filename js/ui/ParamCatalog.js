/**
 * ParamCatalog.js - Known-parameter catalog for on-demand single reads
 *
 * On a high-latency / low-datarate link (LoRa, 900 MHz SiK at 19200, satellite),
 * a full PARAM_REQUEST_LIST is thousands of packets and several minutes. To let
 * the operator pull just the one parameter they care about, the params page needs
 * a list of parameter names *before* any parameter has been downloaded.
 *
 * The catalog is built from three sources, merged and de-duplicated:
 *   1. A built-in seed list of the commonly-tuned ArduPilot parameters, split by
 *      vehicle class so a Plane doesn't show copter-only names.
 *   2. Names learned at runtime — every parameter ever received from a vehicle and
 *      every name seen in a loaded .param file — persisted in localStorage so the
 *      catalog gets richer the more the GCS is used (one full READ ALL on a fast
 *      bench link permanently seeds the catalog for later field use).
 *   3. Whatever is currently in STATE.parameters.
 *
 * Names are never invented: an unknown name typed into the search box can still be
 * requested directly, and if the autopilot answers it is learned from that point on.
 */

const STORAGE_KEY = 'corv.params.learnedNames';
const FAV_KEY = 'corv.params.favorites';

/** Parameters present on every ArduPilot vehicle. */
const COMMON_PARAMS = [
    // Identity / board
    'SYSID_THISMAV', 'SYSID_MYGCS', 'SYSID_ENFORCE',
    'BRD_SAFETYENABLE', 'BRD_SAFETY_DEFLT', 'BRD_SAFETYOPTION', 'BRD_TYPE',
    'BRD_SERIAL_NUM', 'BRD_BOOT_DELAY', 'BRD_OPTIONS', 'BRD_HEAT_TARG',
    'FORMAT_VERSION', 'SCHED_LOOP_RATE', 'SCHED_DEBUG', 'SCHED_OPTIONS',
    // Arming
    'ARMING_CHECK', 'ARMING_REQUIRE', 'ARMING_RUDDER', 'ARMING_ACCTHRESH',
    'ARMING_MIS_ITEMS', 'ARMING_OPTIONS',
    // Battery
    'BATT_MONITOR', 'BATT_CAPACITY', 'BATT_LOW_VOLT', 'BATT_LOW_MAH',
    'BATT_CRT_VOLT', 'BATT_CRT_MAH', 'BATT_FS_LOW_ACT', 'BATT_FS_CRT_ACT',
    'BATT_ARM_VOLT', 'BATT_ARM_MAH', 'BATT_VOLT_PIN', 'BATT_CURR_PIN',
    'BATT_VOLT_MULT', 'BATT_AMP_PERVLT', 'BATT_AMP_OFFSET', 'BATT_LOW_TIMER',
    'BATT_WATT_MAX', 'BATT_SERIAL_NUM', 'BATT_OPTIONS',
    'BATT2_MONITOR', 'BATT2_CAPACITY', 'BATT2_LOW_VOLT',
    // Compass
    'COMPASS_USE', 'COMPASS_USE2', 'COMPASS_USE3', 'COMPASS_ORIENT',
    'COMPASS_DEC', 'COMPASS_AUTODEC', 'COMPASS_LEARN', 'COMPASS_ENABLE',
    'COMPASS_OFS_X', 'COMPASS_OFS_Y', 'COMPASS_OFS_Z',
    'COMPASS_DIA_X', 'COMPASS_DIA_Y', 'COMPASS_DIA_Z',
    'COMPASS_ODI_X', 'COMPASS_ODI_Y', 'COMPASS_ODI_Z',
    'COMPASS_MOT_X', 'COMPASS_MOT_Y', 'COMPASS_MOT_Z', 'COMPASS_MOTCT',
    'COMPASS_PRIO1_ID', 'COMPASS_OFFS_MAX', 'COMPASS_SCALE', 'COMPASS_TYPEMASK',
    // AHRS / EKF
    'AHRS_EKF_TYPE', 'AHRS_GPS_USE', 'AHRS_ORIENTATION', 'AHRS_TRIM_X',
    'AHRS_TRIM_Y', 'AHRS_TRIM_Z', 'AHRS_WIND_MAX', 'AHRS_COMP_BETA',
    'AHRS_GPS_GAIN', 'AHRS_YAW_P', 'AHRS_RP_P', 'AHRS_OPTIONS',
    'EK3_ENABLE', 'EK3_GPS_TYPE', 'EK3_SRC1_POSXY', 'EK3_SRC1_VELXY',
    'EK3_SRC1_POSZ', 'EK3_SRC1_VELZ', 'EK3_SRC1_YAW', 'EK3_ALT_M_NSE',
    'EK3_POS_I_GATE', 'EK3_VEL_I_GATE', 'EK3_HGT_I_GATE', 'EK3_MAG_CAL',
    'EK3_GLITCH_RAD', 'EK3_IMU_MASK', 'EK3_OPTIONS', 'EK3_DRAG_BCOEF_X',
    'EK2_ENABLE',
    // GPS
    'GPS_TYPE', 'GPS_TYPE2', 'GPS_AUTO_SWITCH', 'GPS_AUTO_CONFIG',
    'GPS_NAVFILTER', 'GPS_MIN_ELEV', 'GPS_MIN_DGPS', 'GPS_HDOP_GOOD',
    'GPS_RATE_MS', 'GPS_RATE_MS2', 'GPS_GNSS_MODE', 'GPS_SBAS_MODE',
    'GPS_POS1_X', 'GPS_POS1_Y', 'GPS_POS1_Z', 'GPS_DELAY_MS',
    'GPS_INJECT_TO', 'GPS_BLEND_MASK', 'GPS_PRIMARY', 'GPS_DRV_OPTIONS',
    // IMU
    'INS_GYRO_FILTER', 'INS_ACCEL_FILTER', 'INS_USE', 'INS_USE2', 'INS_USE3',
    'INS_GYRO_CAL', 'INS_TRIM_OPTION', 'INS_ACC_BODYFIX', 'INS_FAST_SAMPLE',
    'INS_HNTCH_ENABLE', 'INS_HNTCH_FREQ', 'INS_HNTCH_BW', 'INS_HNTCH_ATT',
    'INS_HNTCH_REF', 'INS_HNTCH_MODE', 'INS_HNTCH_OPTS', 'INS_LOG_BAT_MASK',
    'INS_NOTCH_ENABLE', 'INS_NOTCH_FREQ', 'INS_NOTCH_BW', 'INS_NOTCH_ATT',
    'INS_ACC1_CALTEMP', 'INS_TCAL1_ENABLE',
    // Logging
    'LOG_BITMASK', 'LOG_BACKEND_TYPE', 'LOG_DISARMED', 'LOG_REPLAY',
    'LOG_FILE_BUFSIZE', 'LOG_FILE_DSRMROT', 'LOG_MAX_FILES', 'LOG_FILE_MB_FREE',
    // Fence / rally
    'FENCE_ENABLE', 'FENCE_TYPE', 'FENCE_ACTION', 'FENCE_ALT_MAX',
    'FENCE_ALT_MIN', 'FENCE_RADIUS', 'FENCE_MARGIN', 'FENCE_TOTAL',
    'FENCE_AUTOENABLE', 'FENCE_OPTIONS', 'FENCE_RET_RALLY', 'FENCE_RET_ALT',
    'RALLY_TOTAL', 'RALLY_LIMIT_KM', 'RALLY_INCL_HOME',
    // Failsafe (shared subset)
    'FS_GCS_ENABL', 'FS_GCS_TIMEOUT', 'FS_EKF_ACTION', 'FS_EKF_THRESH',
    'FS_CRASH_CHECK', 'FS_OPTIONS',
    // RC input
    'RC_SPEED', 'RC_PROTOCOLS', 'RC_OPTIONS', 'RC_OVERRIDE_TIME',
    'RC_FS_TIMEOUT', 'RCMAP_ROLL', 'RCMAP_PITCH', 'RCMAP_THROTTLE', 'RCMAP_YAW',
    'RC1_MIN', 'RC1_MAX', 'RC1_TRIM', 'RC1_REVERSED', 'RC1_DZ', 'RC1_OPTION',
    'RC2_MIN', 'RC2_MAX', 'RC2_TRIM', 'RC2_REVERSED', 'RC2_DZ', 'RC2_OPTION',
    'RC3_MIN', 'RC3_MAX', 'RC3_TRIM', 'RC3_REVERSED', 'RC3_DZ', 'RC3_OPTION',
    'RC4_MIN', 'RC4_MAX', 'RC4_TRIM', 'RC4_REVERSED', 'RC4_DZ', 'RC4_OPTION',
    'RC5_OPTION', 'RC6_OPTION', 'RC7_OPTION', 'RC8_OPTION', 'RC9_OPTION',
    'RC10_OPTION', 'RC11_OPTION', 'RC12_OPTION',
    // Servo output
    'SERVO1_FUNCTION', 'SERVO1_MIN', 'SERVO1_MAX', 'SERVO1_TRIM', 'SERVO1_REVERSED',
    'SERVO2_FUNCTION', 'SERVO2_MIN', 'SERVO2_MAX', 'SERVO2_TRIM', 'SERVO2_REVERSED',
    'SERVO3_FUNCTION', 'SERVO3_MIN', 'SERVO3_MAX', 'SERVO3_TRIM', 'SERVO3_REVERSED',
    'SERVO4_FUNCTION', 'SERVO4_MIN', 'SERVO4_MAX', 'SERVO4_TRIM', 'SERVO4_REVERSED',
    'SERVO5_FUNCTION', 'SERVO6_FUNCTION', 'SERVO7_FUNCTION', 'SERVO8_FUNCTION',
    'SERVO9_FUNCTION', 'SERVO10_FUNCTION', 'SERVO11_FUNCTION', 'SERVO12_FUNCTION',
    'SERVO_AUTO_TRIM', 'SERVO_RATE', 'SERVO_DSHOT_RATE', 'SERVO_BLH_MASK',
    'SERVO_BLH_AUTO', 'SERVO_BLH_TRATE',
    // Serial ports
    'SERIAL0_BAUD', 'SERIAL0_PROTOCOL',
    'SERIAL1_BAUD', 'SERIAL1_PROTOCOL', 'SERIAL1_OPTIONS',
    'SERIAL2_BAUD', 'SERIAL2_PROTOCOL', 'SERIAL2_OPTIONS',
    'SERIAL3_BAUD', 'SERIAL3_PROTOCOL', 'SERIAL3_OPTIONS',
    'SERIAL4_BAUD', 'SERIAL4_PROTOCOL', 'SERIAL5_BAUD', 'SERIAL5_PROTOCOL',
    'SERIAL6_BAUD', 'SERIAL6_PROTOCOL', 'SERIAL7_BAUD', 'SERIAL7_PROTOCOL',
    // Telemetry stream rates — the ones that matter on a slow link
    'SR0_EXTRA1', 'SR0_EXTRA2', 'SR0_EXTRA3', 'SR0_POSITION', 'SR0_RAW_SENS',
    'SR0_EXT_STAT', 'SR0_RC_CHAN', 'SR0_PARAMS', 'SR0_ADSB',
    'SR1_EXTRA1', 'SR1_EXTRA2', 'SR1_EXTRA3', 'SR1_POSITION', 'SR1_RAW_SENS',
    'SR1_EXT_STAT', 'SR1_RC_CHAN', 'SR1_PARAMS', 'SR1_ADSB',
    'SR2_EXTRA1', 'SR2_EXTRA2', 'SR2_EXTRA3', 'SR2_POSITION', 'SR2_PARAMS',
    'SR3_EXTRA1', 'SR3_EXTRA2', 'SR3_EXTRA3', 'SR3_POSITION', 'SR3_PARAMS',
    // Mission / terrain
    'MIS_TOTAL', 'MIS_RESTART', 'MIS_OPTIONS',
    'TERRAIN_ENABLE', 'TERRAIN_FOLLOW', 'TERRAIN_SPACING', 'TERRAIN_MARGIN',
    'TERRAIN_OPTIONS',
    // Rangefinder / flow / mount / camera
    'RNGFND1_TYPE', 'RNGFND1_MIN_CM', 'RNGFND1_MAX_CM', 'RNGFND1_ORIENT',
    'RNGFND1_GNDCLEAR', 'RNGFND1_PIN', 'RNGFND1_SCALING', 'RNGFND1_ADDR',
    'RNGFND2_TYPE', 'RNGFND_LANDING',
    'FLOW_TYPE', 'FLOW_FXSCALER', 'FLOW_FYSCALER', 'FLOW_ORIENT_YAW',
    'MNT1_TYPE', 'MNT1_DEFLT_MODE', 'MNT1_RC_RATE', 'MNT1_PITCH_MIN',
    'MNT1_PITCH_MAX', 'MNT1_YAW_MIN', 'MNT1_YAW_MAX',
    'CAM1_TYPE', 'CAM_TRIGG_TYPE', 'CAM_DURATION', 'CAM_FEEDBACK_PIN',
    // Notify / misc peripherals
    'NTF_LED_BRIGHT', 'NTF_LED_TYPES', 'NTF_BUZZ_TYPES', 'NTF_BUZZ_VOLUME',
    'ADSB_TYPE', 'ADSB_EMIT_TYPE', 'ADSB_LIST_MAX', 'ADSB_LIST_RADIUS',
    'AVD_ENABLE', 'AVD_F_ACTION', 'AVD_F_DIST_XY', 'AVD_F_DIST_Z',
    'MSP_OPTIONS', 'CAN_P1_DRIVER', 'CAN_D1_PROTOCOL',
    'RSSI_TYPE', 'RSSI_ANA_PIN', 'RSSI_PIN_LOW', 'RSSI_PIN_HIGH',
    'SCR_ENABLE', 'SCR_HEAP_SIZE', 'SCR_VM_I_COUNT',
    'STAT_BOOTCNT', 'STAT_FLTTIME', 'STAT_RUNTIME', 'STAT_RESET',
    'GRIP_ENABLE', 'GRIP_TYPE', 'CHUTE_ENABLED', 'CHUTE_TYPE', 'CHUTE_ALT_MIN',
];

/** ArduPlane (fixed wing + QuadPlane). */
const PLANE_PARAMS = [
    // Airspeed
    'ARSPD_TYPE', 'ARSPD_USE', 'ARSPD_RATIO', 'ARSPD_AUTOCAL', 'ARSPD_PIN',
    'ARSPD_OFFSET', 'ARSPD_TUBE_ORDER', 'ARSPD_SKIP_CAL', 'ARSPD_PSI_RANGE',
    'ARSPD_FBW_MIN', 'ARSPD_FBW_MAX', 'ARSPD_OPTIONS', 'ARSPD_WIND_MAX',
    'ARSPD2_TYPE', 'ARSPD2_USE',
    // TECS (speed/height controller)
    'TECS_CLMB_MAX', 'TECS_SINK_MIN', 'TECS_SINK_MAX', 'TECS_TIME_CONST',
    'TECS_THR_DAMP', 'TECS_INTEG_GAIN', 'TECS_VERT_ACC', 'TECS_HGT_OMEGA',
    'TECS_SPD_OMEGA', 'TECS_RLL2THR', 'TECS_SPDWEIGHT', 'TECS_PTCH_DAMP',
    'TECS_LAND_ARSPD', 'TECS_LAND_THR', 'TECS_LAND_SPDWGT', 'TECS_PITCH_MAX',
    'TECS_PITCH_MIN', 'TECS_LAND_SINK', 'TECS_LAND_TCONST', 'TECS_LAND_DAMP',
    'TECS_LAND_PMAX', 'TECS_APPR_SMAX', 'TECS_LAND_SRC', 'TECS_LAND_TDAMP',
    'TECS_LAND_IGAIN', 'TECS_TKOFF_IGAIN', 'TECS_OPTIONS', 'TECS_SYNAIRSPEED',
    // Navigation (L1 and NavEKF-based)
    'NAVL1_PERIOD', 'NAVL1_DAMPING', 'NAVL1_XTRACK_I', 'NAVL1_LIM_BANK',
    'WP_RADIUS', 'WP_LOITER_RAD', 'WP_MAX_RADIUS', 'RTL_RADIUS',
    'ALT_HOLD_RTL', 'ALT_HOLD_FBWCM', 'ALT_OFFSET', 'RTL_AUTOLAND',
    'CRUISE_ALT_FLOOR', 'GLIDE_SLOPE_MIN', 'GLIDE_SLOPE_THR',
    // Attitude controllers (fixed wing)
    'RLL_RATE_P', 'RLL_RATE_I', 'RLL_RATE_D', 'RLL_RATE_FF', 'RLL_RATE_FLTD',
    'RLL_RATE_FLTE', 'RLL_RATE_FLTT', 'RLL_RATE_IMAX', 'RLL_RATE_SMAX',
    'PTCH_RATE_P', 'PTCH_RATE_I', 'PTCH_RATE_D', 'PTCH_RATE_FF', 'PTCH_RATE_FLTD',
    'PTCH_RATE_FLTE', 'PTCH_RATE_FLTT', 'PTCH_RATE_IMAX', 'PTCH_RATE_SMAX',
    'YAW_RATE_ENABLE', 'YAW2SRV_SLIP', 'YAW2SRV_INT', 'YAW2SRV_DAMP',
    'YAW2SRV_RLL', 'YAW2SRV_IMAX',
    'RLL2SRV_TCONST', 'RLL2SRV_RMAX', 'PTCH2SRV_TCONST', 'PTCH2SRV_RMAX_UP',
    'PTCH2SRV_RMAX_DN', 'PTCH2SRV_RLL', 'ACRO_ROLL_RATE', 'ACRO_PITCH_RATE',
    'ACRO_LOCKING', 'ACRO_YAW_RATE',
    // Limits and trim
    'LIM_ROLL_CD', 'LIM_PITCH_MAX', 'LIM_PITCH_MIN',
    'TRIM_THROTTLE', 'TRIM_ARSPD_CM', 'TRIM_PITCH_CD',
    'THR_MIN', 'THR_MAX', 'THR_SLEWRATE', 'THR_SUPP_MAN', 'THR_PASS_STAB',
    'THR_FAILSAFE', 'THR_FS_VALUE', 'THROTTLE_NUDGE', 'THR_MAX_CRUISE',
    'STALL_PREVENTION', 'SCALING_SPEED', 'MIN_GROUNDSPEED',
    'KFF_RDDRMIX', 'KFF_THR2PTCH', 'FLAP_1_PERCNT', 'FLAP_1_SPEED',
    'FLAP_2_PERCNT', 'FLAP_2_SPEED', 'FLAP_SLEWRATE', 'FLAP_IN_CHANNEL',
    'MIXING_GAIN', 'MIXING_OFFSET', 'ELEVON_OUTPUT', 'VTAIL_OUTPUT',
    'RUDDER_ONLY', 'GROUND_STEER_ALT', 'GROUND_STEER_DPS',
    'STEER2SRV_P', 'STEER2SRV_I', 'STEER2SRV_D', 'STEER2SRV_TCONST',
    // Takeoff / landing
    'TKOFF_THR_MAX', 'TKOFF_THR_MINSPD', 'TKOFF_THR_MINACC', 'TKOFF_THR_DELAY',
    'TKOFF_TDRAG_ELEV', 'TKOFF_TDRAG_SPD1', 'TKOFF_ROTATE_SPD', 'TKOFF_LVL_ALT',
    'TKOFF_ALT', 'TKOFF_DIST', 'TKOFF_FLAP_PCNT', 'TKOFF_OPTIONS',
    'TKOFF_ACCEL_CNT', 'TKOFF_THR_MAX_T', 'TKOFF_TIMEOUT',
    'LAND_FLARE_ALT', 'LAND_FLARE_SEC', 'LAND_PITCH_CD', 'LAND_FLAP_PERCNT',
    'LAND_DISARMDELAY', 'LAND_THEN_NEUTRL', 'LAND_ABORT_THR', 'LAND_TYPE',
    'LAND_DS_V_FWD', 'LAND_OPTIONS',
    // Modes and failsafe
    'FLTMODE1', 'FLTMODE2', 'FLTMODE3', 'FLTMODE4', 'FLTMODE5', 'FLTMODE6',
    'FLTMODE_CH', 'INITIAL_MODE', 'FS_SHORT_ACTN', 'FS_SHORT_TIMEOUT',
    'FS_LONG_ACTN', 'FS_LONG_TIMEOUT', 'RTL_CLIMB_MIN', 'FBWB_ELEV_REV',
    'FBWB_CLIMB_RATE', 'CRUISE_HEIGHT_OFF',
    // Soaring / thermalling
    'SOAR_ENABLE', 'SOAR_VSPEED', 'SOAR_MIN_THML_S', 'SOAR_MIN_CRUISE_S',
    'SOAR_ALT_MAX', 'SOAR_ALT_MIN', 'SOAR_ALT_CUTOFF', 'SOAR_POLAR_CD0',
    'SOAR_POLAR_B', 'SOAR_POLAR_K',
    // QuadPlane / VTOL
    'Q_ENABLE', 'Q_FRAME_CLASS', 'Q_FRAME_TYPE', 'Q_M_SPIN_ARM', 'Q_M_SPIN_MIN',
    'Q_M_SPIN_MAX', 'Q_M_PWM_TYPE', 'Q_M_PWM_MIN', 'Q_M_PWM_MAX',
    'Q_M_THST_HOVER', 'Q_M_THST_EXPO', 'Q_M_BAT_VOLT_MAX', 'Q_M_BAT_VOLT_MIN',
    'Q_A_RAT_RLL_P', 'Q_A_RAT_RLL_I', 'Q_A_RAT_RLL_D', 'Q_A_RAT_RLL_FLTD',
    'Q_A_RAT_PIT_P', 'Q_A_RAT_PIT_I', 'Q_A_RAT_PIT_D', 'Q_A_RAT_PIT_FLTD',
    'Q_A_RAT_YAW_P', 'Q_A_RAT_YAW_I', 'Q_A_RAT_YAW_D',
    'Q_A_ANG_RLL_P', 'Q_A_ANG_PIT_P', 'Q_A_ANG_YAW_P', 'Q_A_ACCEL_R_MAX',
    'Q_A_ACCEL_P_MAX', 'Q_A_ACCEL_Y_MAX', 'Q_A_INPUT_TC',
    'Q_P_POSXY_P', 'Q_P_VELXY_P', 'Q_P_VELXY_I', 'Q_P_VELXY_D',
    'Q_P_POSZ_P', 'Q_P_VELZ_P', 'Q_P_ACCZ_P', 'Q_P_ACCZ_I', 'Q_P_ACCZ_D',
    'Q_ANGLE_MAX', 'Q_TRANSITION_MS', 'Q_TRANS_DECEL', 'Q_TRANS_FAIL',
    'Q_ASSIST_SPEED', 'Q_ASSIST_ANGLE', 'Q_ASSIST_ALT', 'Q_ASSIST_DELAY',
    'Q_RTL_MODE', 'Q_RTL_ALT', 'Q_LAND_FINAL_ALT', 'Q_LAND_FINAL_SPD',
    'Q_LAND_SPEED', 'Q_WVANE_ENABLE', 'Q_WVANE_GAIN', 'Q_WP_SPEED',
    'Q_WP_SPEED_UP', 'Q_WP_SPEED_DN', 'Q_WP_ACCEL', 'Q_WP_RADIUS',
    'Q_VFWD_GAIN', 'Q_VFWD_ALT', 'Q_TILT_MASK', 'Q_TILT_TYPE', 'Q_TILT_RATE_UP',
    'Q_TILT_RATE_DN', 'Q_TILT_MAX', 'Q_OPTIONS', 'Q_THR_MAX_PWM',
    'Q_FWD_THR_USE', 'Q_TAILSIT_ENABLE', 'Q_TAILSIT_MOTMX', 'Q_TAILSIT_ANGLE',
];

/** ArduCopter / heli. */
const COPTER_PARAMS = [
    'FRAME_CLASS', 'FRAME_TYPE',
    'ATC_RAT_RLL_P', 'ATC_RAT_RLL_I', 'ATC_RAT_RLL_D', 'ATC_RAT_RLL_FF',
    'ATC_RAT_RLL_FLTD', 'ATC_RAT_RLL_FLTE', 'ATC_RAT_RLL_FLTT', 'ATC_RAT_RLL_IMAX',
    'ATC_RAT_PIT_P', 'ATC_RAT_PIT_I', 'ATC_RAT_PIT_D', 'ATC_RAT_PIT_FF',
    'ATC_RAT_PIT_FLTD', 'ATC_RAT_PIT_FLTE', 'ATC_RAT_PIT_FLTT', 'ATC_RAT_PIT_IMAX',
    'ATC_RAT_YAW_P', 'ATC_RAT_YAW_I', 'ATC_RAT_YAW_D', 'ATC_RAT_YAW_FF',
    'ATC_RAT_YAW_FLTE', 'ATC_RAT_YAW_FLTT', 'ATC_RAT_YAW_IMAX',
    'ATC_ANG_RLL_P', 'ATC_ANG_PIT_P', 'ATC_ANG_YAW_P', 'ATC_ANG_LIM_TC',
    'ATC_ACCEL_R_MAX', 'ATC_ACCEL_P_MAX', 'ATC_ACCEL_Y_MAX', 'ATC_INPUT_TC',
    'ATC_SLEW_YAW', 'ATC_THR_MIX_MAN', 'ATC_THR_MIX_MIN', 'ATC_THR_MIX_MAX',
    'ATC_RATE_R_MAX', 'ATC_RATE_P_MAX', 'ATC_RATE_Y_MAX',
    'PSC_POSXY_P', 'PSC_VELXY_P', 'PSC_VELXY_I', 'PSC_VELXY_D',
    'PSC_VELXY_FLTD', 'PSC_VELXY_FLTE', 'PSC_VELXY_IMAX',
    'PSC_POSZ_P', 'PSC_VELZ_P', 'PSC_VELZ_I', 'PSC_VELZ_D',
    'PSC_ACCZ_P', 'PSC_ACCZ_I', 'PSC_ACCZ_D', 'PSC_ACCZ_FLTD', 'PSC_ACCZ_IMAX',
    'PSC_ANGLE_MAX', 'PSC_JERK_XY', 'PSC_JERK_Z',
    'WPNAV_SPEED', 'WPNAV_SPEED_UP', 'WPNAV_SPEED_DN', 'WPNAV_ACCEL',
    'WPNAV_ACCEL_Z', 'WPNAV_RADIUS', 'WPNAV_RFND_USE', 'WPNAV_JERK',
    'LOIT_SPEED', 'LOIT_ACC_MAX', 'LOIT_BRK_ACCEL', 'LOIT_BRK_DELAY',
    'LOIT_BRK_JERK', 'LOIT_ANG_MAX',
    'MOT_SPIN_ARM', 'MOT_SPIN_MIN', 'MOT_SPIN_MAX', 'MOT_THST_EXPO',
    'MOT_THST_HOVER', 'MOT_BAT_VOLT_MAX', 'MOT_BAT_VOLT_MIN', 'MOT_BAT_CURR_MAX',
    'MOT_PWM_TYPE', 'MOT_PWM_MIN', 'MOT_PWM_MAX', 'MOT_HOVER_LEARN',
    'MOT_YAW_HEADROOM', 'MOT_SAFE_DISARM', 'MOT_OPTIONS',
    'PILOT_SPEED_UP', 'PILOT_SPEED_DN', 'PILOT_ACCEL_Z', 'PILOT_THR_BHV',
    'PILOT_THR_FILT', 'PILOT_TKOFF_ALT', 'PILOT_Y_RATE', 'PILOT_Y_EXPO',
    'ANGLE_MAX', 'ACRO_RP_RATE', 'ACRO_Y_RATE', 'ACRO_BAL_ROLL', 'ACRO_BAL_PITCH',
    'ACRO_TRAINER', 'ACRO_RP_EXPO', 'ACRO_THR_MID',
    'RTL_ALT', 'RTL_ALT_FINAL', 'RTL_SPEED', 'RTL_CONE_SLOPE', 'RTL_CLIMB_MIN',
    'RTL_LOIT_TIME', 'RTL_ALT_TYPE',
    'LAND_SPEED', 'LAND_SPEED_HIGH', 'LAND_ALT_LOW', 'LAND_REPOSITION',
    'THR_DZ', 'SUPER_SIMPLE', 'SIMPLE', 'CIRCLE_RADIUS', 'CIRCLE_RATE',
    'CIRCLE_OPTIONS', 'AUTOTUNE_AXES', 'AUTOTUNE_AGGR', 'AUTOTUNE_MIN_D',
    'FLTMODE1', 'FLTMODE2', 'FLTMODE3', 'FLTMODE4', 'FLTMODE5', 'FLTMODE6',
    'FLTMODE_CH', 'INITIAL_MODE',
    'FS_THR_ENABLE', 'FS_THR_VALUE', 'FS_GCS_ENABLE', 'FS_VIBE_ENABLE',
    'DISARM_DELAY', 'GND_EFFECT_COMP', 'THROW_TYPE', 'THROW_MOT_START',
    'PRX1_TYPE', 'AVOID_ENABLE', 'AVOID_MARGIN', 'AVOID_BEHAVE',
    'H_SV_MAN', 'H_COL_MIN', 'H_COL_MAX', 'H_RSC_MODE', 'H_RSC_SETPOINT',
];

/** ArduRover / boat. */
const ROVER_PARAMS = [
    'FRAME_CLASS', 'FRAME_TYPE',
    'CRUISE_SPEED', 'CRUISE_THROTTLE', 'SPEED_TURN_GAIN', 'TURN_RADIUS',
    'WP_SPEED', 'WP_RADIUS', 'WP_OVERSHOOT', 'WP_PIVOT_ANGLE', 'WP_PIVOT_RATE',
    'WP_PIVOT_DELAY', 'WP_ACCEL', 'WP_JERK',
    'ATC_STR_RAT_P', 'ATC_STR_RAT_I', 'ATC_STR_RAT_D', 'ATC_STR_RAT_FF',
    'ATC_STR_RAT_MAX', 'ATC_STR_ACC_MAX', 'ATC_STR_ANG_P',
    'ATC_SPEED_P', 'ATC_SPEED_I', 'ATC_SPEED_D', 'ATC_SPEED_FF',
    'ATC_ACCEL_MAX', 'ATC_DECEL_MAX', 'ATC_BRAKE', 'ATC_STOP_SPEED',
    'ATC_TURN_MAX_G', 'ATC_BAL_P', 'ATC_BAL_I', 'ATC_BAL_D',
    'NAVL1_PERIOD', 'NAVL1_DAMPING',
    'MOT_PWM_TYPE', 'MOT_PWM_FREQ', 'MOT_SAFE_DISARM', 'MOT_THR_MIN',
    'MOT_THR_MAX', 'MOT_SLEWRATE', 'MOT_VEC_THR_BASE', 'MOT_SPD_SCA_BASE',
    'MODE1', 'MODE2', 'MODE3', 'MODE4', 'MODE5', 'MODE6', 'MODE_CH',
    'INITIAL_MODE', 'PILOT_STEER_TYPE', 'FS_ACTION', 'FS_TIMEOUT',
    'FS_THR_ENABLE', 'FS_THR_VALUE', 'FS_GCS_ENABLE',
    'RTL_SPEED', 'LOIT_TYPE', 'LOIT_RADIUS', 'LOIT_SPEED_GAIN',
    'SAIL_ENABLE', 'SAIL_ANGLE_MIN', 'SAIL_ANGLE_MAX', 'SAIL_ANGLE_IDEAL',
    'SIMPLE_TYPE', 'TURN_MAX_G', 'DOCK_SPEED', 'AUTO_KICKSTART',
];

/** ArduSub. */
const SUB_PARAMS = [
    'FRAME_CONFIG', 'JS_GAIN_DEFAULT', 'JS_GAIN_MAX', 'JS_GAIN_MIN',
    'JS_GAIN_STEPS', 'JS_LIGHTS_STEPS', 'JS_THR_GAIN', 'JS_CAM_TILT_STEP',
    'PILOT_SPEED_UP', 'PILOT_SPEED_DN', 'SURFACE_DEPTH', 'FS_PRESS_ENABLE',
    'FS_PRESS_MAX', 'FS_TEMP_ENABLE', 'FS_TEMP_MAX', 'FS_LEAK_ENABLE',
    'ATC_RAT_RLL_P', 'ATC_RAT_PIT_P', 'ATC_RAT_YAW_P',
    'PSC_POSZ_P', 'PSC_VELZ_P', 'PSC_ACCZ_P', 'PSC_ACCZ_I',
];

const VEHICLE_SETS = {
    Plane: PLANE_PARAMS,
    Copter: COPTER_PARAMS,
    Rover: ROVER_PARAMS,
    Sub: SUB_PARAMS,
};

// ── Learned names (persisted) ────────────────────────────────────────────────

let learnedNames = null;   // Set<string>, lazily loaded
let favorites = null;      // Set<string>, lazily loaded

function loadSet(key) {
    try {
        const raw = localStorage.getItem(key);
        if (!raw) return new Set();
        const arr = JSON.parse(raw);
        return new Set(Array.isArray(arr) ? arr : []);
    } catch (e) {
        return new Set();
    }
}

function saveSet(key, set) {
    try {
        localStorage.setItem(key, JSON.stringify([...set].sort()));
    } catch (e) {
        // Quota exceeded or storage disabled — the catalog still works in-memory.
        console.warn('[ParamCatalog] Could not persist', key, e.message);
    }
}

function ensureLoaded() {
    if (!learnedNames) learnedNames = loadSet(STORAGE_KEY);
    if (!favorites) favorites = loadSet(FAV_KEY);
}

/**
 * Record parameter names seen from a vehicle or a .param file so they show up in
 * the catalog on the next (possibly offline) session.
 * @param {Iterable<string>} names
 */
export function learnNames(names) {
    ensureLoaded();
    let added = 0;
    for (const raw of names) {
        const name = String(raw || '').trim().toUpperCase();
        if (!name || name.length > 16) continue;
        if (!learnedNames.has(name)) { learnedNames.add(name); added++; }
    }
    if (added) saveSet(STORAGE_KEY, learnedNames);
    return added;
}

/** Number of learned (non-built-in) names currently stored. */
export function learnedCount() {
    ensureLoaded();
    return learnedNames.size;
}

/** Drop every learned name (built-ins and favorites are untouched). */
export function clearLearned() {
    ensureLoaded();
    learnedNames.clear();
    saveSet(STORAGE_KEY, learnedNames);
}

// ── Favorites ────────────────────────────────────────────────────────────────

export function isFavorite(name) {
    ensureLoaded();
    return favorites.has(name);
}

export function toggleFavorite(name) {
    ensureLoaded();
    if (favorites.has(name)) favorites.delete(name);
    else favorites.add(name);
    saveSet(FAV_KEY, favorites);
    return favorites.has(name);
}

export function getFavorites() {
    ensureLoaded();
    return [...favorites].sort();
}

// ── Catalog assembly ─────────────────────────────────────────────────────────

/**
 * Group label for a parameter, used for the group filter.
 * SERVO1_FUNCTION -> SERVO, EK3_ENABLE -> EK, Q_A_RAT_RLL_P -> Q, FLTMODE1 -> FLTMODE
 */
export function groupOf(name) {
    const i = name.indexOf('_');
    const head = i > 0 ? name.slice(0, i) : name;
    const stripped = head.replace(/\d+$/, '');
    return stripped.length >= 2 ? stripped : head;
}

/**
 * Full catalog for a vehicle class: built-ins for that class + common + every
 * learned name + everything already downloaded in this session.
 * @param {string} vehicleName - 'Plane' | 'Copter' | 'Rover' | 'Sub'
 * @param {Map<string,object>} [loadedParams] - STATE.parameters
 * @returns {string[]} sorted, de-duplicated names
 */
export function getCatalog(vehicleName, loadedParams = null) {
    ensureLoaded();
    const set = new Set(COMMON_PARAMS);
    const vehicleSet = VEHICLE_SETS[vehicleName] || PLANE_PARAMS;
    for (const n of vehicleSet) set.add(n);
    for (const n of learnedNames) set.add(n);
    if (loadedParams) for (const n of loadedParams.keys()) set.add(n);
    return [...set].sort();
}

/** Distinct group labels present in a catalog, sorted. */
export function getGroups(names) {
    const set = new Set(names.map(groupOf));
    return [...set].sort();
}

/** True if the name is part of the built-in seed list (not learned). */
export function isBuiltin(name) {
    return COMMON_PARAMS.includes(name)
        || PLANE_PARAMS.includes(name)
        || COPTER_PARAMS.includes(name)
        || ROVER_PARAMS.includes(name)
        || SUB_PARAMS.includes(name);
}
