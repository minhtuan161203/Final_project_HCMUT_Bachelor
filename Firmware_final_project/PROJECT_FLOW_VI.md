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

## Phan bo sung cho luan van: Firmware Overview va Brainstorming Flow

Phan nay duoc viet theo goc nhin "kien truc su ky thuat + mentor", nghia la khong di theo kieu doc source tung dong ma tap trung vao:

- he thong van hanh theo lop nhu the nao
- moi lop trao doi du lieu ra sao
- chung ta da debug theo tu duy nao de di tu "hien tuong" toi "giai phap chot"

### 0. Mental model tong quat

Neu can mo ta ngan gon toan bo firmware trong mot hinh:

```text
GUI / USB command
-> while(1) parser va dispatcher
-> cap nhat parameter / flag / mode

FPGA heartbeat + feedback
-> EXTI interrupt vao STM32
-> current loop 16 kHz
-> speed / position loop chia tan
-> FOC output
-> PWM / pha dien ap

Encoder / current / Vdc / temp
-> duoc FPGA dong bo va dua vao STM32 theo cung nhip
```

Hay xem he thong nay nhu mot servo drive co 2 the gioi song song:

- the gioi "lenh va cau hinh" chay trong `while(1)`
- the gioi "thuc thi dieu khien" chay trong ISR duoc kich boi nhip dong bo tu FPGA

### 1. Synchronization Layer: Heartbeat giua FPGA va STM32

Ban chat cua he thong khong phai la STM32 tu chay mot timer doc lap de dieu khien motor, ma la STM32 dang "song dong bo" voi FPGA.

#### 1.1 Vi sao can FPGA heartbeat

FPGA dung vai tro:

- tong hop / chot feedback current
- chot du lieu encoder
- dua Vdc, nhiet do, fault signal vao mot mien thoi gian on dinh
- kich STM32 bang xung dong bo de moi lan tinh FOC deu dung tren "mot mau du lieu da on dinh"

Dieu nay giai quyet mot van de rat thuc te:

- current, encoder, fault, va PWM update phai cung mot nhip
- neu tung khoi tu chay le nhip, current loop se tinh tren feedback cu trong khi PWM lai dang la chu ky moi

#### 1.2 Handshake qua TIM8 co y nghia gi

TIM8, dac biet nhanh feedback pulse qua CH4, dong vai tro "toi da nhan xung va dang song dung nhip voi FPGA".

Tu duy dung o day la:

- FPGA khong chi "phat xung danh thuc" STM32
- FPGA con can thay mot xung phan hoi de biet STM32 van dang theo kip nhip he thong

Neu xung feedback nay mat:

- FPGA co the coi he thong dong bo bi loi
- kich hoat co che failsafe / watchdog noi bo
- giam nhip hoac doi che do phat xung
- dan den STM32 thay tan so ngat tu 16 kHz rot xuong muc rat thap

Noi cach khac, TIM8 CH4 khong phai mot chi tiet "phu", ma la mot phan cua hop dong dong bo thoi gian thuc giua 2 bo xu ly.

#### 1.3 Y nghia kien truc

Do co lop handshake nay, firmware co duoc 3 thu:

1. Current loop luon tinh tren bo mau feedback nhat quan.
2. PWM update xay ra cung phase voi feedback encoder/current.
3. Loi mat dong bo duoc bieu hien thanh "he thong cham di" thay vi cho phep FOC chay tren du lieu rac.

### 2. Control Hierarchy: Cascaded Loop

Cau truc dieu khien cua he thong la:

```text
Position Loop
-> tao ra Speed Reference
-> Speed Loop
-> tao ra Iq Reference
-> Current Loop
-> tao ra Vd / Vq
-> InvPark / InvClarke / PWM
```

#### 2.1 Current loop

Current loop la tang trong cung va nhanh nhat.

Nhiem vu:

- nhan `Id_ref`, `Iq_ref`
- so sanh voi `Id`, `Iq` do tu feedback
- tao `Vd`, `Vq`
- dua qua bien doi nghich de phat duty cho 3 pha

Tang nay tra loi cau hoi:

> Muon tao ra vector dong dien nao trong stator ngay luc nay?

#### 2.2 Speed loop

Speed loop dung tang current nhu mot "co bap".

Nhiem vu:

- nhan `Speed_ref`
- so voi `ActSpeed`
- tao `Iq_ref`

Tang nay tra loi cau hoi:

> Muon rotor quay nhanh hon hay cham hon thi can bao nhieu moment?

#### 2.3 Position loop

Position loop dung tang speed nhu mot "co che di chuyen".

Nhiem vu:

- nhan `TargetPosition`
- so voi `ActPosition`
- tao `Speed_ref`

Tang nay tra loi cau hoi:

> Muon di den vi tri mong muon thi can chay theo huong nao, nhanh den dau?

#### 2.4 Luong du lieu qua cac tang

Luong du lieu trong trang thai FOC on dinh co the nhin nhu sau:

```text
Target Position
-> Position PI + VFF
-> Cmd Speed
-> Speed PI
-> Iq Ref
-> Current PI
-> Vq
-> PWM
-> Rotor chuyen dong
-> Encoder / speed feedback
-> quay nguoc lai Position va Speed loop
```

`Id_ref` thuong duoc giu gan 0 trong surface PMSM de tap trung moment vao truc Q.

### 3. Operational Stages: tu Idle den FOC on dinh

#### 3.1 Idle

Trang thai nay he thong:

- da boot xong
- da load parameter tu flash
- PWM o trang thai an toan
- co the nhan lenh tu GUI

Day la luc firmware "san sang", nhung chua duoc phep dieu khien motor.

#### 3.2 Servo ON / Arming

Khi nhan `Servo ON`, firmware khong lap tuc dong vong FOC, ma di qua chuoi chuan bi:

- current sensor offset calibration
- kiem tra dieu kien fault
- xac nhan encoder / alignment policy

Muc tieu la dam bao moi phep do co y nghia truoc khi bat dau sinh moment.

#### 3.3 Alignment

Day la giai doan xac lap moc 0 giua:

- goc co khi encoder
- khung tham chieu dien cua FOC

Neu bo qua buoc nay:

- Park / Clarke van chay
- current PI van tao ra `Iq`
- nhung `Iq` do co the khong nam dung truc sinh moment

Alignment chinh la luc firmware tra loi cau hoi:

> Khi code goi day la `theta = 0`, rotor dang nam o dau tren truc D?

#### 3.4 Speed / Position FOC Running

Sau khi alignment xong:

- runtime theta duoc dung
- current loop bat dau sinh vector dien ap co y nghia
- speed loop hoac position loop duoc dong vao theo mode da chon

Luc nay he thong moi that su buoc vao che do servo.

#### 3.5 Fault / Stop / Recovery

Neu co fault:

- firmware cat moment
- dua he thong ve trang thai an toan
- giu monitor va log de GUI doc lai

Dieu quan trong la:

- fault khong chi la "bao loi"
- no la co che dam bao tat ca tang control dung lai theo thu tu an toan

## Brainstorming va Debugging Journey

Phan nay khong viet theo dang "bug list", ma theo dang nhat ky cac vu an ky thuat. Moi vu an deu co 4 buoc:

1. Hien tuong
2. Gia thuyet
3. Thuc nghiem va loai tru
4. Giai phap chot

### Case 1: Mau thuan he toa do va quy uoc chieu

#### Hien tuong

- vua dong vong kin thi dong co giat manh
- co luc vua vao closed-loop la voc toc, mat kiem soat
- co luc `Iq` co ve dung dau nhung chieu quay lai nguoc voi mong doi

#### Gia thuyet

Gia thuyet trung tam la:

> He thong chua thong nhat "chieu duong" giua encoder, speed estimate, va torque sinh boi truc Q.

Loi kieu nay rat de xay ra khi:

- encoder tang count theo chieu nay
- nhung field dien cua FOC lai coi do la chieu nguoc
- hoac `+Iq` dang sinh torque theo chieu am trong quy uoc co khi

#### Thuc nghiem va loai tru

Flow suy nghi da di qua cac buoc:

1. Tach current loop khoi speed loop bang mode test.
2. Tiem `Iq_ref` duong / am nho de xem rotor nhich ve huong nao.
3. So sanh chieu nhich co khi voi dau cua `ActSpeed`.
4. Thu cac to hop mapping current feedback de loai tru loi ADC/sign.
5. Dung test chieu quay tu dong de thong ke:
   - mo-men sinh ra theo huong nao
   - encoder dem tang hay giam

Qua cac buoc nay, van de duoc he thong hoa thanh mot quy trinh:

- khong sua tay theo cam tinh
- ma de firmware tu quan sat "chi can biet minh ra lenh torque duong, rotor va encoder phan ung theo dau nao"

#### Giai phap chot

Giai phap chot la mot phuong phap tu dong xac dinh chieu quay:

- firmware tao mot kich thich co huong ro rang
- quan sat phan ung encoder / speed
- tu dong dat quy uoc chieu cho toan he thong

Gia tri cua huong tiep can nay khong nam o viec sua mot bug cu the, ma o cho:

> "Chieu duong" tro thanh mot khoi cau hinh co the duoc he thong tu hoc va tu khoa, thay vi phu thuoc vao viec doi day pha hay sua dau trong code mot cach cam tinh.

### Case 2: Sai lech khung tham chieu dien

#### Hien tuong

- motor khong sinh mo-men toi uu
- rotor bi khung, nong, hoac dung o nhung vi tri co dinh
- `Iq` nhin co ve bam ref nhung motor khong quay nhu mong doi
- 3 pha co luc nhin nhu dong DC dung yên

#### Gia thuyet

Gia thuyet o day la:

> Goc co khi cua encoder da co, nhung goc dien dung cho Park/InvPark chua trung voi truc D that cua rotor.

Noi cach khac:

- firmware biet "rotor dang o dau" theo co khi
- nhung chua biet "theta nao moi la theta dung de chieu dong vao truc D/Q"

#### Thuc nghiem va loai tru

Flow brainstorm cua vu an nay da di qua nhieu tang:

1. Tach speed loop ra khoi bai toan.
2. Dung `Id tuning` de tim goc ma:
   - `Id` len dung dau
   - `Iq` gan 0
3. Thuc hien cac bai test quay theta mo:
   - rotating theta current test
   - rotating theta voltage test
4. So sanh:
   - neu voltage mode quay muot ma current mode khong muot -> nghi current feedback frame
   - neu ca hai deu sai -> nghi phase order / electrical frame / offset
5. Thu sweep goc de tim diem mo-men tot nhat va loai tru cac gia thuyet sai dau / sai frame.

Day la vu an quan trong nhat ve mat FOC, vi no day chung ta tu "cam thay motor khong ngot" toi mot ket luan ro rang:

- FOC khong chi can current PI dung
- ma con can khung tham chieu dien dung

#### Giai phap chot

Giai phap chot la:

- xac lap chinh xac offset giua goc encoder va goc dien
- khoa offset nay vao runtime
- dam bao truc D va truc Q vuong goc dung nghia vat ly

Khi offset duoc khoa dung:

- `Id` moi that su la truc tu thong
- `Iq` moi that su la truc sinh moment
- current PI va speed loop moi co y nghia co khi

### Case 3: Su bat on dinh tan so ngat

#### Hien tuong

- he thong dang duoc thiet ke chay 16 kHz
- nhung khi thieu mot thanh phan init, tan so ngat roi xuong khoang 3 kHz
- cac loop phia tren nhin nhu van chay, nhung cam giac he thong "mat nhip"

#### Gia thuyet

Gia thuyet o day khong con la bug dieu khien don thuan, ma la bug dong bo he thong:

> FPGA dang co co che watchdog / failsafe noi bo, va no can mot xung phan hoi de tin rang STM32 van theo kip nhip.

Khi mat xung nay, FPGA khong con tin cay chu trinh dong bo hien tai nua, nen ha nhip hoac doi co che phat xung.

#### Thuc nghiem va loai tru

Flow suy nghi da di theo huong:

1. So sanh truong hop co / khong co khoi init TIM8 CH4.
2. Quan sat tan so EXTI thuc te thay doi theo su co mat cua feedback pulse.
3. Loai tru cac gia thuyet phu:
   - khong phai current PI
   - khong phai parser USB
   - khong phai chi rieng peripheral timer noi bo
4. Noi lai van de bang ngon ngu kien truc:
   - "day khong phai timer bug"
   - "day la giao thuc dong bo giua FPGA va STM32 dang bi vo"

#### Giai phap chot

Giai phap chot la thiet lap co che feedback pulse day du:

- FPGA cap heartbeat cho STM32
- TIM8 CH4 tra lai xung xac nhan
- ca 2 ben cung song tren mot hop dong thoi gian thuc ro rang

Ket qua cua cach nhin nay la:

> Tan so 16 kHz khong con la mot con so "mong muon", ma la mot trang thai dong bo can duoc bao toan bang handshake.

## Tong ket phuong phap luan

Neu tong hop lai toan bo hanh trinh phat trien firmware FOC, phuong phap luan xuyen suot la:

1. Tach bai toan lon thanh tung tang:
   - sync
   - frame
   - feedback
   - current
   - speed
   - position
2. Moi khi co hien tuong la, uu tien tach loop ngoai de kiem tra loop trong.
3. Khong sua dau / offset / mapping theo cam giac; phai co test de quyet dinh.
4. Moi bug quan trong deu duoc chuyen hoa thanh:
   - mot quy trinh do
   - mot diagnostic mode
   - hoac mot co che tu dong adapt

Do la ly do firmware cuoi cung khong chi "chay duoc", ma con co the giai thich duoc:

- no dong bo voi ai
- no dieu khien theo tang nao
- no dua tren quy uoc chieu nao
- va tai sao cac quyet dinh debug lai dan toi cau truc hien tai

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
