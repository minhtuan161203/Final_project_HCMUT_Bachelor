# ASD04 RevE LV2 - Project Flow va Kien truc tong the

## Muc tieu cua tai lieu

Tai lieu nay duoc viet de giup ban:

- hieu project nay chay nhu the nao tu luc boot den luc dong co dang run
- thay ro ranh gioi giua `while(1)` va vong ngat ngoai
- biet cac module chinh, struct chinh, va API nao goi API nao
- co mot "ban do doc source" de khong bi lac giua rat nhieu bien global

Tai lieu nay duoc tong hop chu yeu tu cac file:

- `Src/main.c`
- `Library/USBComunication.c`
- `Library/StateMachine.c`
- `Library/Parameter.c`
- `Library/PWM.c`
- `Library/Flash.c`
- `Library/filter_th.c`
- cac header trong `Library/*.h`

## Luu y quan trong truoc khi doc

Workspace hien tai co day du header cho cac module control:

- `CurrentControl.h`
- `VelocityControl.h`
- `PositionControl.h`
- `JOG.h`
- `fft.h`
- `AutoTuning.h`

nhung khong thay source `.c` tuong ung trong thu muc `Library/`.

Vi vay:

- flow tong the va quan he API van doc duoc rat ro tu `main.c`, `USBComunication.c`, `Parameter.c` va cac header
- phan thuat toan ben trong PI/current/speed/position/auto tuning duoc mo ta theo API va du lieu vao/ra, khong di sau vao implementation chi tiet

## 1. Ban chat cua project nay

Neu tom tat trong 1 cau:

> Day la firmware STM32F407 cho servo driver, trong do giao tiep command/monitor chay o `while(1)`, con vong dieu khien thoi gian thuc chay trong `HAL_GPIO_EXTI_Callback()` duoc kich boi xung tu FPGA/driver.

Mental model don gian:

```text
PC GUI
-> USB CDC
-> USB_ProcessData() trong while
-> set command / set flag / set parameter / set state --> tất cả command thao tác chỉ là set các cờ chờ các chu kì ngắt sau thực hiện

Xung dong bo tu FPGA/driver
-> EXTI interrupt
-> HAL_GPIO_EXTI_Callback()
-> doc feedback
-> tinh current loop / speed loop / position loop
-> cap nhat PWM
-> check fault / update state
```

Hay nho 2 y nay:

1. `while(1)` khong lam control loop.
2. `HAL_GPIO_EXTI_Callback()` moi la noi "thuc thi that".

## 2. Cau truc thu muc nen quan tam

```text
ASD04_RevE_LV2/
|- Src/
|  |- main.c
|  |- stm32f4xx_it.c
|  |- usbd_cdc_if.c
|  |- usb_device.c
|
|- Inc/
|  |- main.h
|  |- stm32f4xx_it.h
|  |- usbd_cdc_if.h
|
|- Library/
|  |- USBComunication.c/h
|  |- StateMachine.c/h
|  |- Parameter.c/h
|  |- PWM.c/h
|  |- Flash.c/h
|  |- filter_th.c/h
|  |- circular_buffer.c/h
|  |- CurrentControl.h
|  |- VelocityControl.h
|  |- PositionControl.h
|  |- PIControl.h
|  |- JOG.h
|  |- fft.h
|  |- AutoTuning.h
|  |- define.h
|
|- Drivers/
|  |- STM32 HAL + CMSIS
|
|- Middlewares/
|  |- USB Device stack
|
|- MDK-ARM/
|  |- project Keil, build output, map file
```

## 3. Vai tro cua tung lop trong he thong

### 3.1 Lop hardware / register map

File: `Library/define.h`

No dinh nghia cac dia chi memory map den FPGA:

- `REG_CURRENT_PHASE_U`
- `REG_CURRENT_PHASE_V`
- `REG_TEMPARATURE_SENSOR`
- `REG_DC_BUS_VOLTAGE`
- `REG_ENCODER_ID`
- `REG_ENCODER_RX_WORD0..6`
- `REG_FPGA_ERROR`

STM32 truy cap cac register nay qua vung nho bat dau tu:

```c
#define FPGA_START_ADDR (uint32_t)0x64000000
```

Nghia la:

- feedback encoder, current, Vdc, temperature khong doc truc tiep tu ADC/peripheral STM32
- STM32 doc thong qua FPGA/driver bang memory-mapped interface

### 3.2 Lop command va monitor

File chinh:

- `Library/USBComunication.c`
- `Src/usbd_cdc_if.c`
- `Library/circular_buffer.c`

Nhiem vu:

- nhan frame command tu PC qua USB CDC
- parse, check CRC, ACK
- doi state / doi parameter / bat trace / bat tuning / bat JOG
- gui nguoc lai data monitor, error, trace, tuning graph

### 3.3 Lop state machine

File chinh:

- `Library/StateMachine.c`

Trang thai chinh:

- `IDLE`
- `OFFSET_CALIB`
- `START`
- `RUN`
- `STOP`
- `FAULT_NOW`
- `FAULT_OVER`

State machine nay duoc command loop va ISR cung thao tac.

### 3.4 Lop control thoi gian thuc

Noi chinh:

- `Src/main.c`
- ham `HAL_GPIO_EXTI_Callback()`

Nhiem vu:

- doc feedback
- tinh goc dien
- Clark/Park
- current control
- speed/position/JOG/auto tuning
- nghich bien doi sang dien ap pha
- phat duty ra PWM
- check fault

### 3.5 Lop persistent parameter

File:

- `Library/Flash.c`
- `Library/USBComunication.c`

Nhiem vu:

- luu `DriverParameter[16]`
- luu `MotorParameter[32]`
- doc lai luc boot

## 4. Cac bien global quan trong nhat

Trong `main.c` co mot nhom bien global duoc dung nhu "trang thai toan cuc" cua firmware:

- `Parameter` : feedback va trang thai sensor/encoder hien tai
- `StateMachine` : state hien tai cua driver
- `Current_Sensor` : thong tin calib current va threshold overcurrent
- `CurrentCtrl` : current loop + FOC current state
- `SpeedCtrl` : speed loop
- `PosCtrl` : position loop
- `DriverParameter[16]` : param cua drive
- `MotorParameter[32]` : param cua motor/encoder/current loop
- `USB_Comm` : flags giao tiep USB
- `CurrentTuning`, `SpeedTuning`, `AtTuning` : tuning state
- `Commutation`, `CommuProcess` : can chinh hall/commutation
- `Trace_Data`, `fft_handle`, `Bode_ng` : trace va measurement
- `EnableRun` : co cho phep current/PWM chay hay khong
- `FaultCode` : ma loi tong hop

Ban nen xem nhung bien nay nhu cac "mailbox" duoc chia se giua:

- command loop trong `while(1)`
- control ISR trong `HAL_GPIO_EXTI_Callback()`

## 5. Flow khoi dong tu reset den luc san sang nhan lenh

Flow trong `main()`:

```text
HAL_Init()
-> Init_Parameter(&Parameter)
-> SystemClock_Config()
-> init GPIO/TIM/FSMC/USB/ADC
-> FLASH_Read(DriverParameter)
-> FLASH_Read(MotorParameter)
-> khoi tao PWM trung tinh 50%
-> ack fault pin / enable ADC read pin
-> UpdateDriverParameter()
-> UpdateMotorParameter()
-> CurrentControl_SetParam()
-> init Bode signal generator
-> vao while(1)
```

Y nghia:

- sau boot, firmware doc parameter da luu trong flash
- nap parameter do vao cac object control
- dua PWM ve muc trung tinh
- sau do ngoi trong `while(1)` de cho command tu PC

## 6. Vong `while(1)` dung de lam gi

Trong `main.c`:

```c
while (1)
{
    USB_ProcessData(&USB_Comm);
    USB_TransmitData(&USB_Comm);
}
```

Day la "communication loop".

No khong chay PI current, khong tinh theta, khong generate control theo chu ky.

No chi lam 2 viec:

1. `USB_ProcessData()`
   - boc frame USB
   - giai ma command
   - set bien global / set state / set flag

2. `USB_TransmitData()`
   - gui du lieu monitor
   - gui trace/FFT/tuning data
   - gui ma loi

## 7. USB flow chi tiet

### 7.1 Chieu nhan

Flow nhan USB:

```text
USB packet tu PC
-> CDC_Receive_FS()
-> USB_ReceiveData()
-> circBufPush() vao raw circular buffer

while(1)
-> USB_ProcessData()
-> circBufPop()
-> rap frame
-> check CRC
-> xu ly command
```

Vai tro tung ham:

- `CDC_Receive_FS()` trong `Src/usbd_cdc_if.c`
  - duoc USB middleware goi khi co packet moi
  - khong xu ly command tai day
  - chi day data vao `USB_ReceiveData()`

- `USB_ReceiveData()` trong `Library/USBComunication.c`
  - push byte vao `rawCircBuffer`

- `USB_ProcessData()`
  - moi la noi parse frame that su
  - frame co dang:

```text
STX | SYN/ACK | SIZE | CMD | DATA... | CRC | ETX
```

### 7.2 Chieu gui

`USB_TransmitData()` gui cac loai data sau:

- error packet
- driver parameter
- motor parameter
- tuning data
- FFT/Bode data
- trace data

Ham tao frame gui la `SendData()`.

## 8. Command loop tac dong len control nhu the nao

Nguyen tac:

- USB command khong dieu khien PWM truc tiep
- USB command chi set state/flag/parameter
- ISR se thay doi nay o lan ngat tiep theo va thuc thi

Vi du:

### 8.1 Servo ON

```text
PC gui CMD_SERVO_ON
-> USB_ProcessData()
-> STM_NextState(OFFSET_CALIB)

ngat tiep theo
-> HAL_GPIO_EXTI_Callback()
-> state OFFSET_CALIB
-> CalibrateCurrentSensor()
-> xong thi START
-> tiep theo la RUN
```

### 8.2 Servo OFF

```text
PC gui CMD_SERVO_OFF
-> USB_ProcessData()
-> STM_NextState(STOP)

ngat tiep theo
-> state STOP
-> reset position/speed/current controller
-> State -> IDLE
-> SwitchOffPWM()
```

### 8.3 Ghi parameter

```text
PC gui CMD_WRITE_DRIVER / CMD_WRITE_MOTOR
-> USB_ProcessData()
-> cap nhat DriverParameter[] / MotorParameter[]
-> goi UpdateDriverParameter() / UpdateMotorParameter()
```

Tu thoi diem do:

- vong dieu khien su dung gain/tham so moi
- neu muon luu xuong flash thi gui them `CMD_WRITE_TO_FLASH`

### 8.4 JOG

```text
PC gui lenh JOG
-> USB_ProcessData()
-> set JOG.Enable / JOG_Mode / Direction / CmdSpeed / Distance ...

ngat RUN tiep theo
-> HAL_GPIO_EXTI_Callback()
-> block JOG trong state RUN
-> JOG_Speed_Calc() hoac JOG_Position_Calc()
-> PositionControl_Calc() neu can
-> SpeedCtrl.CmdSpeed
-> VelocityControl_Calc()
-> CurrentCtrl.fIq_ref
-> CurrentControl_Cacl()
-> PWM
```

## 9. Nguon xung dieu khien thoi gian thuc

GPIO duoc cau hinh lam EXTI:

- `ECAT_IRQ_Pin` : falling edge
- `ECAT_SYNC0_Pin` : falling edge
- `iMeasureIsr_Pin` : rising edge

Trong `stm32f4xx_it.c`:

```text
EXTI0_IRQHandler     -> HAL_GPIO_EXTI_IRQHandler(ECAT_IRQ_Pin)
EXTI1_IRQHandler     -> HAL_GPIO_EXTI_IRQHandler(ECAT_SYNC0_Pin)
EXTI15_10_IRQHandler -> HAL_GPIO_EXTI_IRQHandler(iMeasureIsr_Pin)
```

Sau do HAL deu don ve cung mot callback:

```c
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
```

### Luu y rat quan trong

Hien tai `HAL_GPIO_EXTI_Callback()` trong `main.c` khong check `GPIO_Pin`.

Dieu nay co nghia la:

- ve mat code, ca 3 nguon EXTI tren deu co the chay chung logic control
- ve mat y tuong he thong, pin control loop dung co ve la `iMeasureIsr_Pin`
- neu ban xem xung FPGA la trigger control loop, thi do la mo hinh dung
- nhung code hien tai khong khoa lai bang `if (GPIO_Pin == iMeasureIsr_Pin)`

Day la mot diem can biet khi debug timing.

## 10. `HAL_GPIO_EXTI_Callback()` la trai tim cua project

Day la control loop ISR.

No lam rat nhieu viec, co the chia thanh 8 lop nhu sau.

### 10.1 Commutation mode neu dang alignment / auto commutation

Neu `CommuProcess.Enable == 1`:

- giam sat hall/index
- tang `DelayCounter`
- moi mot so chu ky thi doi `StepAngle`
- xac dinh huong quay
- tinh so pole pair
- tinh hall offset
- update `MotorParameter`
- ket thuc commutation va co the chuyen sang `STOP`

Flow nay dung khi:

- auto commutation
- auto tuning can alignment rotor

### 10.2 Tinh theta dien `Parameter.fTheta`

Neu khong trong commutation dac biet, ISR se suy ra `fTheta` dua tren:

- loai encoder (`MotorParameter[MOTOR_ENCODER_ID]`)
- huong current control (`MOTOR_CURRENT_CTRL_DIRECTION`)
- hall offset
- hall table forward/reverse cho mot so loai encoder

Day la buoc noi feedback encoder/hall vao FOC.

### 10.3 Doc feedback tu FPGA / driver

Ham:

```text
GetParameter(&Parameter, MotorParameter)
```

No doc:

- `REG_ENCODER_RX_WORD0..6`
- `REG_DC_BUS_VOLTAGE`
- `REG_CURRENT_PHASE_U`
- `REG_CURRENT_PHASE_V`
- `REG_TEMPARATURE_SENSOR`

Roi cap nhat:

- `EncSingleTurn`
- `DeltaPos`
- `fPosition`
- `HallSensor`
- `fVdc`
- `fIabc[3]`
- `fTemparature`

Nghia la `Parameter` chinh la anh chup feedback moi nhat cua he thong.

### 10.4 Tinh toc do thuc te

Sau `GetParameter()`, ISR tinh:

- `Parameter.fActSpeed`

dua tren:

- chenh lech vi tri giua 2 chu ky
- `Ui16ControlFrequency`
- mode don vi `u8IsPulseOrRPM`

### 10.5 Current loop / FOC

Flow current loop:

```text
Parameter.fIabc
-> CurrentCtrl.fIu/fIv/fIw
-> ClarkAndParkTransform()
-> CurrentControl_Cacl()
-> InvParkAndInvClarkTransform()
-> SinPWM()
-> GeneratePWM()
```

Y nghia:

- current act vao he quy chieu `dq`
- PI dong current tao `Vd/Vq`
- doi nguoc ve he `abc`
- bien thanh duty PWM

Neu `EnableRun == 0` thi firmware dua PWM ve `0.5 / 0.5 / 0.5`.

### 10.6 State machine layer trong ISR

Sau current loop co block xu ly state:

#### `OFFSET_CALIB`

- `SwitchOnPWM()`
- `GeneratePWM(0.5, 0.5, 0.5)`
- `CalibrateCurrentSensor()`
- du 5000 mau thi tinh offset current
- chuyen sang `START`

#### `START`

- `SwitchOnPWM()`
- chuyen ngay sang `RUN`

#### `RUN`

Tai day moi co speed loop / position loop / JOG / tuning / trace / FFT.

Mot phan quan trong:

- `SpeedCtrl.Counter++`
- cu 2 lan ISR moi chay nhom speed/position logic mot lan

Nghia la:

- current loop chay o tan so ngat
- speed loop chay cham hon current loop

#### `STOP`

- `EnableRun = 0`
- reset `PosCtrl`, `SpeedCtrl`, `CurrentCtrl`
- chuyen ve `IDLE`

### 10.7 Outer loop: speed / position / JOG / tuning

Trong `RUN`, logic outer loop co cac nhanh:

#### a. Speed tuning

- `Speed_Tuning()`

#### b. Auto tuning

- `Current_AutoTuning_Process()`
- `Speed_Position_AutoTuning_Process()`

#### c. FFT / Bode

- `generate_inp_fft()`
- ghi du lieu vao `RecordTable1/2`

#### d. JOG

- speed JOG:
  - `JOG_Speed_Calc()`
  - ghi vao `SpeedCtrl.CmdSpeed`

- position JOG:
  - `JOG_Position_Calc()`
  - `PositionControl_Calc()`
  - ket qua dua thanh `SpeedCtrl.CmdSpeed`

#### e. Speed loop binh thuong

Neu khong tuning, khong JOG, khong commutation:

```text
VelocityControl_Calc(&SpeedCtrl, SpeedCtrl.CmdSpeed, Parameter.fActSpeed)
-> CurrentCtrl.fIq_ref = SpeedCtrl.fOutputValue
```

Noi day la outer loop speed -> current.

### 10.8 Filter, trace, FFT va fault

Cuoi ISR con cac lop phu:

- filter torque ref:
  - `Calc_Second_Order_Lowpass_Filter()`
  - `Cal_Matlab_Notch_Filter()`

- FFT/trace capture:
  - lay du lieu vao `RecordTable1`
  - lay du lieu vao `RecordTable2`

- fault:
  - `CheckCurrentPhaseFault()`
  - `CheckTempFault()`
  - `STM_FaultProcessing()`

Neu vao fault:

- reset controller
- `USB_Comm.SendError = 1`
- `SwitchOffPWM()`

## 11. State machine: cach no chuyen trang thai

State machine duoc viet kha gon:

```text
IDLE -> OFFSET_CALIB
OFFSET_CALIB -> START hoac STOP
START -> RUN hoac STOP
RUN -> STOP
STOP -> IDLE
bat ky state nao cung co the vao FAULT_NOW khi fault xuat hien
FAULT_NOW -> FAULT_OVER khi fault het
FAULT_OVER -> IDLE khi user ACK fault
```

Ban chat cua no:

- bao ve trinh tu bat servo
- tach logic "cho phep chay" khoi logic command
- tao diem chung de fault co the cat dong co

## 12. Parameter map: `DriverParameter[]` va `MotorParameter[]`

### 12.1 DriverParameter

Mang `DriverParameter[16]` chua:

- control mode
- position gain
- speed gain
- feed-forward
- acceleration/deceleration
- maximum speed
- speed unit
- torque filter frequency

Ham ap dung:

```text
UpdateDriverParameter(DriverParameter)
```

No lam cac viec:

- nap gain cho `SpeedCtrl`
- set `SpeedCtrl.SpeedUnit`
- set `Parameter.u8IsPulseOrRPM`
- tinh JOG acc/dec step
- neu dang position mode thi `PositionControl_SetParam()`

### 12.2 MotorParameter

Mang `MotorParameter[32]` chua:

- current/torque/rated data
- encoder type
- encoder resolution
- maximum speed
- current PI gains
- pole pair
- hall offset
- hall lookup table

Ham ap dung:

```text
UpdateMotorParameter(MotorParameter)
```

No lam cac viec:

- ghi encoder ID sang register FPGA `REG_ENCODER_ID`
- cap nhat gain current PI
- cap nhat `Parameter.u8PolePair`
- cap nhat threshold overcurrent
- cap nhat encoder resolution
- cap nhat output limit speed loop

## 13. Hardware flow: tu feedback den PWM

Day la flow quan trong nhat cua project:

```text
FPGA/driver cap nhat register memory map
-> xung ngat den STM32
-> HAL_GPIO_EXTI_Callback()
-> GetParameter()
   -> doc current, voltage, temp, encoder
-> tinh theta dien
-> ClarkAndParkTransform()
-> CurrentControl_Cacl()
-> VelocityControl_Calc() / PositionControl_Calc() neu can
-> InvParkAndInvClarkTransform()
-> SinPWM()
-> GeneratePWM()
-> TIM8 CCR1/2/3
-> cong suat ra 3 pha motor
```

Trong do:

- `Parameter.c` lo doc feedback
- `CurrentControl` lo FOC current
- `VelocityControl` la outer speed loop
- `PositionControl` la outer position loop
- `PWM.c` lo xuat duty ra TIM8

## 14. Module map: module nao dung de lam gi

### `Parameter`

Vai tro:

- doc feedback tu FPGA
- xu ly encoder theo tung protocol
- tinh vi tri / delta position
- doc current, Vdc, temperature
- current offset calibration
- fault sensor layer

API chinh:

- `Init_Parameter()`
- `GetParameter()`
- `CalibrateCurrentSensor()`
- `CheckCurrentPhaseFault()`
- `CheckTempFault()`

### `CurrentControl`

Vai tro:

- current PI cho `Id` va `Iq`
- Clark/Park va bien doi nguoc
- tinh duty PWM tu `Vd/Vq`

API chinh:

- `CurrentControl_SetParam()`
- `CurrentControl_Reset()`
- `CurrentControl_Cacl()`
- `ClarkAndParkTransform()`
- `InvParkAndInvClarkTransform()`
- `SinPWM()`

### `VelocityControl`

Vai tro:

- speed PI
- tao `Iq_ref`

API chinh:

- `VelocityControl_SetParam()`
- `VelocityControl_Reset()`
- `VelocityControl_Calc()`
- `Speed_Tuning()`

### `PositionControl`

Vai tro:

- position P + feed-forward
- tao speed command cho speed loop

API chinh:

- `PositionControl_SetParam()`
- `PositionControl_Reset()`
- `PositionControl_Calc()`

### `PWM`

Vai tro:

- bat/tat PWM bridge
- cap nhat duty TIM8

API chinh:

- `SwitchOnPWM()`
- `SwitchOffPWM()`
- `GeneratePWM()`

### `USBComunication`

Vai tro:

- parser command tu PC
- ACK/CRC/frame
- read/write parameter
- trace, tuning, FFT, fault packet

API chinh:

- `USB_ReceiveData()`
- `USB_ProcessData()`
- `USB_TransmitData()`
- `SendData()`
- `UpdateDriverParameter()`
- `UpdateMotorParameter()`

### `StateMachine`

Vai tro:

- dieu huong chu trinh song cua servo

API chinh:

- `STM_Init()`
- `STM_NextState()`
- `STM_FaultProcessing()`
- `STM_GetState()`
- `STM_FaultAcknowledged()`

### `Flash`

Vai tro:

- doc/ghi parameter xuong flash noi

API chinh:

- `FLASH_Read()`
- `FLASH_Write()`

### `filter_th`

Vai tro:

- low-pass va notch filter cho torque/speed analysis

API chinh:

- `Init_Second_Order_Lowpass_Filter()`
- `Calc_Second_Order_Lowpass_Filter()`
- `Init_Matlab_Notch_Filter()`
- `Cal_Matlab_Notch_Filter()`

### `JOG`

Vai tro:

- tao profile JOG speed/position

API chinh:

- `JOG_Speed_Calc()`
- `JOG_Position_Calc()`

### `fft`

Vai tro:

- tao kich thich sine/pulse de do dap ung
- luu du lieu cho FFT/Bode

API chinh:

- `generate_sine_init()`
- `generate_inp_fft()`

### `AutoTuning`

Vai tro:

- tu dong tuning current/speed/position
- relay feedback
- commutation sequence phuc vu tuning

API chinh:

- `AutoTuning_Reset()`
- `Next_Tuning_State()`
- `Current_AutoTuning_Process()`
- `Speed_Position_AutoTuning_Process()`

## 15. Cac command quan trong nhat tu PC

Trong `USBComunication.h` co nhieu command, nhung command thuc su tac dong manh den flow gom:

- `CMD_SERVO_ON`
- `CMD_SERVO_OFF`
- `CMD_WRITE_DRIVER`
- `CMD_WRITE_MOTOR`
- `CMD_WRITE_TO_FLASH`
- `CMD_ACK_FAULT`
- `CMD_APPLY_MJOG`
- `CMD_APPLY_JOGPROGRAM`
- `CMD_SERVO_JOGPROGRAM`
- `CMD_START_MJOG_INC`
- `CMD_START_MJOG_DEC`
- `CMD_STOP_MJOG`
- `CMD_START_STUNING`
- `CMD_START_CTUNING`
- `CMD_START_AUTOTUNING_T`
- `CMD_START_COMMUTATION_NEW`
- `CMD_APPLY_TRACE`
- `CMD_APPLY_BODE_OPEN_CLOSE_LOOP`

### Mot phat hien quan trong

Enum command co dinh nghia:

- `CMD_START_TORQUECONTROL`
- `CMD_START_SPEEDCONTROL`
- `CMD_START_POSITIONCONTROL`

nhung trong `USB_ProcessData()` hien tai khong thay case xu ly rieng cho cac command nay.

Dieu nay goi y rang:

- project khong chuyen mode chay bang cac command tren
- thay vao do, mode duoc quyet dinh boi `DriverParameter[CONTROL_MODE]`
- roi sau do ban bat servo/JOG/tuning tuong ung

## 16. Trace, FFT, Bode va tuning duoc dat o dau trong flow

Day la nhom tinh nang "measurement / diagnostic / commissioning".

### 16.1 Trace

- duoc cau hinh boi USB command
- ISR lay mau dinh ky va ghi vao `RecordTable1`
- `USB_TransmitData()` gui tung chunk ve PC

### 16.2 FFT / Bode

- USB bat che do
- ISR tao input kich thich bang `generate_inp_fft()`
- dong thoi ghi output response vao buffer
- `USB_TransmitData()` gui du lieu ve PC

### 16.3 Current/Speed/Auto tuning

- USB set `Enable`, counter, flag gui data
- ISR thuc thi tuning state machine
- du lieu ket qua duoc luu vao `RecordTable1/2`
- `USB_TransmitData()` gui ve GUI

## 17. Luu tru flash va dong doi parameter

Flow luu parameter:

```text
PC gui CMD_WRITE_DRIVER / CMD_WRITE_MOTOR
-> cap nhat mang RAM
-> UpdateDriverParameter() / UpdateMotorParameter()

PC gui CMD_WRITE_TO_FLASH
-> FLASH_Write(PAGE_ADDRESS_DRIVER_PARAMETER, DriverParameter, ...)
-> FLASH_Write(PAGE_ADDRESS_MOTOR_PARAMETER, MotorParameter, ...)
```

Flow nap lai khi boot:

```text
main()
-> FLASH_Read(driver)
-> FLASH_Read(motor)
-> UpdateDriverParameter()
-> UpdateMotorParameter()
```

Nghia la:

- mang RAM moi la ban dang duoc system su dung ngay
- flash chi la noi luu ben vung

## 18. Thu tu nen doc source neu muon hieu nhanh

De onboard nhanh, nen doc theo thu tu sau:

1. `Library/define.h`
   - de biet register map, enum parameter, tan so loop

2. `Library/StateMachine.h` va `StateMachine.c`
   - de hieu vong doi servo

3. `Src/main.c`
   - doc `main()`
   - doc `HAL_GPIO_EXTI_Callback()`

4. `Library/Parameter.h` va `Parameter.c`
   - de hieu feedback den tu dau, encoder parser ra sao

5. `Library/USBComunication.h` va `USBComunication.c`
   - de hieu GUI dieu khien firmware nhu the nao

6. `Library/CurrentControl.h`, `VelocityControl.h`, `PositionControl.h`
   - de thay input/output cua tung vong loop

7. `Library/JOG.h`, `fft.h`, `AutoTuning.h`, `filter_th.h`
   - de hieu cac tinh nang commissioning va test

## 19. Ban do call chain quan trong nhat

### 19.1 Call chain bat servo

```text
CMD_SERVO_ON
-> USB_ProcessData()
-> STM_NextState(OFFSET_CALIB)
-> HAL_GPIO_EXTI_Callback()
-> CalibrateCurrentSensor()
-> STM_NextState(START)
-> HAL_GPIO_EXTI_Callback()
-> SwitchOnPWM()
-> STM_NextState(RUN)
```

### 19.2 Call chain run binh thuong theo speed mode

```text
HAL_GPIO_EXTI_Callback()
-> GetParameter()
-> tinh fTheta
-> ClarkAndParkTransform()
-> VelocityControl_Calc()
-> CurrentCtrl.fIq_ref = SpeedCtrl.fOutputValue
-> CurrentControl_Cacl()
-> InvParkAndInvClarkTransform()
-> SinPWM()
-> GeneratePWM()
```

### 19.3 Call chain position mode

```text
HAL_GPIO_EXTI_Callback()
-> GetParameter()
-> PositionControl_Calc()
-> SpeedCtrl.CmdSpeed
-> VelocityControl_Calc()
-> CurrentCtrl.fIq_ref
-> CurrentControl_Cacl()
-> PWM
```

### 19.4 Call chain fault

```text
HAL_GPIO_EXTI_Callback()
-> CheckCurrentPhaseFault()
-> CheckTempFault()
-> STM_FaultProcessing()
-> state FAULT_NOW
-> reset controller
-> USB_Comm.SendError = 1
-> SwitchOffPWM()
```

### 19.5 Call chain doc command USB

```text
CDC_Receive_FS()
-> USB_ReceiveData()
-> circBufPush()

while(1)
-> USB_ProcessData()
-> parse frame
-> command handler
-> set state/flag/parameter
```

## 20. Nhung diem de gay nham lan khi debug

### 20.1 `while(1)` khong phai noi chay control

Neu ban print log o `while(1)` va thay command da vao, dieu do chua co nghia la PWM da doi.

PWM chi doi o ISR.

### 20.2 Co do tre 1 chu ky ngat giua command va tac dong that

USB command set state/flag truoc.
ISR o lan sau moi dung state/flag do.

### 20.3 `HAL_GPIO_EXTI_Callback()` dang qua nhieu viec

Ham nay vua:

- doc feedback
- current loop
- speed loop
- position loop
- JOG
- tuning
- FFT
- trace
- fault
- state machine

nen neu debug timing, day la noi uu tien so 1.

### 20.4 Nhieu bien global duoc dung chung giua `while` va ISR

Vi du:

- `StateMachine`
- `USB_Comm`
- `DriverParameter`
- `MotorParameter`
- `JOG`
- `AtTuning`
- `EnableRun`

Code hien tai khong co co che lock ro rang. Trong thuc te embedded dieu nay thuong dua vao:

- update khong qua thuong xuyen
- kich thuoc ghi nho
- chon thoi diem command hop ly

### 20.5 Co ten file / include khong dong nhat hoa toan

Vi du:

- `USBComunication` viet thieu mot chu `m`
- `Temparature` thay vi `Temperature`
- `CurrentControl_Cacl` thay vi `Calc`

Day la van de naming, khong phai logic.

## 21. Ket luan ngan gon

Neu ban muon nho project nay bang mot so cau ngan:

- `main()` chi boot he thong va chay communication loop.
- USB command chi la lop "ra lenh".
- `HAL_GPIO_EXTI_Callback()` moi la lop "thi hanh dieu khien".
- `Parameter` la nguon feedback tu FPGA/driver.
- `VelocityControl` va `PositionControl` tao reference cho `CurrentControl`.
- `CurrentControl` tao duty, `PWM` day duty ra TIM8.
- `StateMachine` giu cho trinh tu servo on/off/fault khong bi vo.
- `DriverParameter` va `MotorParameter` la nguon cau hinh trung tam cua toan bo he thong.

---

Neu ban muon mo rong tai lieu nay, huong tiep theo hop ly nhat la:

1. ve so do sequence cho `Servo ON -> RUN -> Fault -> ACK -> IDLE`
2. ve so do data-flow cho `Position -> Speed -> Current -> PWM`
3. tach rieng mot file danh cho `USB protocol` va bang command
4. tach rieng mot file danh cho `encoder/feedback decoding`

