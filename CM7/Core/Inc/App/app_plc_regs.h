#ifndef APP_PLC_REGS_H
#define APP_PLC_REGS_H

#define PLC_IP_ADDR      "192.168.1.100"
#define PLC_MODBUS_PORT  502
#define PLC_UNIT_ID      1

#define HR_SERVO_HIZ          0
#define HR_SERVO_HIZ_GELEN    1
#define HR_SERVO_HIZ_GIDEN    2
#define HR_SERVO_TORK         3
#define HR_SERVO_TORK_GELEN    4
#define HR_SERVO_TORK_GIDEN    5
#define HR_SERVO_MEVCUT_HIZ   6

#define HR_SERVO_MESAFE            (40311 - 40001)
#define HR_EKRAN_SERVO_SABIT_HIZ   (40313 - 40001)
#define HR_EKRAN_SERVO_SABIT_TORK  (40314 - 40001)

#define HR_GERCEK_METRE_RAW   (42011 - 40001)
#define HR_HIZ_M_SN_RAW       (42029 - 40001)
#define HR_MAX_HIZ_M_SN_RAW   (42037 - 40001)

#define COIL_EKRAN_METRE_AKTIF     (5 - 1)
#define COIL_EKRAN_YON_SECIMI      (6 - 1)
#define COIL_METRE_SIFIRLAMA       (7 - 1)
#define COIL_EKRAN_DEGISKEN_MOD    (8 - 1)
#define COIL_ISLEM_BITTI           (11 - 1)

#define COIL_EKRAN_STOP       1
#define COIL_EKRAN_START      2
#define COIL_SISTEM_STARTLI   3

#endif
