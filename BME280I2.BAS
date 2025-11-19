REM =====================================
REM BME280 class library
REM 
REM calib const
STATIC PRIVATE ADDR, t_fine
STATIC PRIVATE dig_T1, dig_T2, dig_T3
STATIC PRIVATE dig_P1, dig_P2, dig_P3
STATIC PRIVATE dig_P4, dig_P5, dig_P6
STATIC PRIVATE dig_P7, dig_P8, dig_P9
STATIC PRIVATE dig_H1, dig_H2, dig_H3
STATIC PRIVATE dig_H4, dig_H5, dig_H6
STATIC PRIVATE th, E4, E5, E6
STATIC PRIVATE var1#, var2#
STATIC PRIVATE raw_t, raw_p, raw_h, hum#
STATIC PRIVATE DAT
STATIC PUBLIC TEMP#,HUMI#,PRESS#

REM --------------------------------------
REM Initialize and calibration
REM
METHOD INIT
  DIM DAT(7)
  ADDR = ARGS(1)
  I2C 100
  REM SOFT RESET
  I2CWRITE ADDR,$E0,$B6
  DELAYMS 2
  I2CWRITE ADDR, $F2, $01 :REM ctrl_hum: oversampleingx1
  I2CWRITE ADDR, $F4, $27 :REM ctrl_meas: oversampling temp x1, press x1, mode normal(3)
  I2CWRITE ADDR, $F5, $20 :REM config:duration62.5ms,filter off
  
  DELAYMS 20

REM ------------------------------------REM BME280calibration
REM calibration
  VAR th,t
  DIM t(23)

  REM Read Calibration data of temperature
  I2CREADDATA ADDR, t, 24, $88

  dig_T1 = PEEK(t) + PEEK(t+1) * 256
  dig_T2 = PEEK(t+2) + PEEK(t+3) * 256
  IF dig_T2 >= 32768 THEN dig_T2 = dig_T2 - 65536
  dig_T3 = PEEK(t+4) + PEEK(t+5) * 256
  IF dig_T3 >= 32768 THEN dig_T3 = dig_T3 - 65536

  REM Read calibration data of  presssure
  dig_P1 = PEEK(t+6) + PEEK(t+7) * 256
  dig_P2 = PEEK(t+8) + PEEK(t+9) * 256
  IF dig_P2 >= 32768 THEN dig_P2 = dig_P2 - 65536
  dig_P3 = PEEK(t+10) + PEEK(t+11) * 256
  IF dig_P3 >= 32768 THEN dig_P3 = dig_P3 - 65536
  dig_P4 = PEEK(t+12) + PEEK(t+13) * 256
  IF dig_P4 >= 32768 THEN dig_P4 = dig_P4 - 65536
  dig_P5 = PEEK(t+14) + PEEK(t+15) * 256
  IF dig_P5 >= 32768 THEN dig_P5 = dig_P5 - 65536
  dig_P6 = PEEK(t+16) + PEEK(t+17) * 256
  IF dig_P6 >= 32768 THEN dig_P6 = dig_P6 - 65536
  dig_P7 = PEEK(t+18) + PEEK(t+19) * 256
  IF dig_P7 >= 32768 THEN dig_P7 = dig_P7 - 65536
  dig_P8 = PEEK(t+20) + PEEK(t+21) * 256
  IF dig_P8 >= 32768 THEN dig_P8 = dig_P8 - 65536
  dig_P9 = PEEK(t+22) + PEEK(t+23) * 256
  IF dig_P9 >= 32768 THEN dig_P9 = dig_P9 - 65536

  REM Read calibration data of  Humidity
  I2CREADDATA ADDR, t, 1, $A1
  dig_H1 = PEEK(t)

  I2CREADDATA ADDR, t, 7, $E1
  dig_H2 = PEEK(t) + PEEK(t+1) * 256
  IF dig_H2 > 32767 THEN dig_H2 = dig_H2 - 65536
  dig_H3 = PEEK(t+2)
REM H4/H5
  E4 = PEEK(t+3)
  E5 = PEEK(t+4)
  E6 = PEEK(t+5)
  dig_H4 = E4 << 4 OR E5 And $0F
  dig_H5 = E6 << 4 OR E5 >> 4
  dig_H6 = PEEK(t+6)
  IF dig_H6 >= 128 THEN dig_H6 = dig_H6 - 256
RETURN


REM =====================================
METHOD GETDATA

REM Read RAW data
  I2CREADDATA ADDR, DAT, 8, $F7
  REM raw data
  raw_p = PEEK(DAT)   << 12 OR PEEK(DAT+1) << 4 OR PEEK(DAT+2) >> 4
  raw_t = PEEK(DAT+3) << 12 OR PEEK(DAT+4) << 4 OR PEEK(DAT+5) >> 4
  raw_h = PEEK(DAT+6) <<  8 OR PEEK(DAT+7)

REM -------------------------------------
REM Calculate t_fine
  VAR var1#,var2#

  var1# =  (FLOAT#(raw_t) / 16384  - FLOAT#(dig_T1) / 1024) * FLOAT#(dig_T2)
  var2# =  (FLOAT#(raw_t) / 131072 - FLOAT#(dig_T1) / 8192) * (FLOAT#(raw_t) / 131072 - FLOAT#(dig_T1) / 8192) * FLOAT#(dig_T3)
  t_fine# = var1# + var2#

REM --------------------------------------
REM Calculate temperature
  TEMP# =  t_fine#  / 5120

REM --------------------------------------
REM Calculate pressure

  REM --- compensate 補正
  var1# = t_fine# / 2 - 64000
  var2# = var1# * var1# * FLOAT#(dig_P6) / 32768
  var2# = var2# + var1# * FLOAT#(dig_P5) * 2
  var2# = var2# / 4 + FLOAT#(dig_P4) * 65536
  var1# = (FLOAT#(dig_P3) * var1# * var1# / 524288 + FLOAT#(dig_P2) * var1#)/ 524288
  var1# = (1 + var1# / 32768) * FLOAT#(dig_P1)
  IF var1# = 0 THEN
    p# = 0
  ELSE
    p# = 1048576 - FLOAT#(raw_p)
    p# = (p# - var2# / 4096) * 6250 / var1#
    var1# = FLOAT#(dig_P9) * p# * p# / 2147483648
    var2# = p# * FLOAT#(dig_P8) / 32768
    p# = p# + (var1# + var2# + FLOAT#(dig_P7)) / 16
  ENDIF
  PRESS# = p# / 100    

REM --------------------------------------
REM Calculate humidity 
  VAR hum#

  hum# = t_fine# - 76800
  hum# = (FLOAT#(raw_h) - (FLOAT#(dig_H4) * 64 + FLOAT#(dig_H5) / 16384 * hum#)) * (FLOAT#(dig_H2) / 65536 * (1 + FLOAT#(dig_H6) / 67108864 * hum# * (1 + FLOAT#(dig_H3) / 67108864 * FLOAT#(hum))))
  hum# = hum# * (1 - FLOAT#(dig_H1) * hum# / 524288)

  IF hum# > 100 THEN hum# = 100
  IF hum# < 0   THEN hum# = 0

  HUMI# = hum#
RETURN
