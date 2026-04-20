# GUI USB Command Flow

## Muc dich

Tai lieu nay giai thich luong giao tiep giua:

- User
- GUI tren PC
- USB CDC / COM port
- Firmware STM32
- Control loop trong driver

Muc tieu la de ban nhin ro:

- bam nut nao tren GUI thi gui command gi
- frame USB co dang nhu the nao
- firmware nhan frame o dau
- ACK va data response di nguoc lai ra sao
- can tach `USB_ReceiveData()` va `USB_ProcessData()` the nao o buoc tiep theo

## Phan bo sung cho luan van: Vai tro cua GUI trong hanh trinh debug FOC

Tai lieu nay ban dau tap trung vao protocol va USB command flow. Tuy nhien, trong qua trinh phat trien FOC, GUI con dong vai tro lon hon nhieu:

- la "bang dieu khien thuc nghiem"
- la noi tach loop de debug
- la cong cu de xac minh gia thuyet ve sign, direction, electrical frame va synchronization

Noi cach khac:

> GUI khong chi la noi bam nut gui lenh, ma la lop giao tiep de bien cac gia thuyet debug thanh bai test co the lap lai.

### 0. Vai tro cua GUI trong methodology chung

Trong hanh trinh phat trien firmware, GUI da duoc dung nhu mot lop "orchestration" cho cac muc tieu sau:

1. Tach bai toan lon thanh bai test nho.
2. Giu firmware khong bi sua tay lien tuc cho moi lan do.
3. Quan sat ket qua theo thoi gian thuc qua Trend / Scope / Snapshot.
4. Dong bo giua:
   - command dang gui
   - mode dang chay
   - feedback dang do

Vi vay, moi nut bam quan trong tren GUI nen duoc xem nhu mot "experiment trigger", khong chi la mot command giao tiep.

### 1. GUI nhu mot lop protocol adapter

GUI dam nhan viec:

- chuyen thao tac nguoi dung thanh frame nhi phan
- dong goi parameter thanh payload co cau truc
- gui dung command ID cho firmware
- nhan ACK / monitor / trace / error de xac minh firmware da o dung trang thai

Neu firmware la "bo nao", thi GUI la:

- bang dieu khien thao tac
- va cung la bang quan sat ket qua

Do do, trong luan van, GUI can duoc mo ta nhu mot thanh phan giup thao tac va xac minh, khong chi la phan phu de demo.

### 2. GUI nhu cong cu tach loop de debug

Trong cac vu an debug FOC, GUI da giup tao ra cac che do thuc nghiem ma neu chi dua vao firmware thuan se rat kho theo doi:

#### 2.1 Tach current loop khoi speed loop

Bang cac mode tuning va diagnostic, GUI cho phep:

- ep `Id_ref` / `Iq_ref`
- khoa `theta`
- chay rotating-theta current test
- chay rotating-theta voltage test

Y nghia:

- current loop co the duoc test doc lap
- speed loop khong lam nhieu ket qua current loop
- direction / electrical frame duoc phan tich ro hon

#### 2.2 Tach speed loop khoi position loop

Khi position loop chua on dinh, GUI cho phep:

- chay speed mode doc lap
- dat target speed, limit, ramp
- xem trend `Cmd Speed`, `Act Speed`, `Speed Error`

Y nghia:

- xac nhan inner loop da on truoc khi dong outer loop
- giam xac suat nham "bug position" voi "bug speed"

#### 2.3 Tinh truu tuong cua deadband, gain va target

Sau khi position loop duoc nang cap:

- GUI da cho phep nhap target theo `deg`
- co relative jog
- co slider gui command theo kieu "commit on release"

Dieu nay rat quan trong ve mat methodology:

- nguoi test tu duy theo goc co khi, khong theo counts
- target duoc gui mot cach co kiem soat
- giam hien tuong spam command gay nhieu cho outer loop

### 3. GUI nhu cong cu xac minh tung gia thuyet debug

Day la cach GUI da dong hanh cung firmware trong tung nhom van de:

#### 3.1 Van de sign / direction

GUI dong vai tro:

- phat command torque / speed co dau ro rang
- hien `ActSpeed`
- ghi log va trend de so dau giua lenh va phan ung

Tu do, chung ta khong can doan:

- `+Iq` co sinh torque duong hay khong
- chieu quay encoder co cung quy uoc voi speed khong

GUI tro thanh noi "chung minh" gia thuyet, khong chi hien so.

#### 3.2 Van de electrical frame / dq alignment

GUI dong vai tro:

- chuyen nhanh giua tuning mode va normal FOC
- ghi trend `Id`, `Iq`, `Vd`, `Vq`
- cho phep capture va so sanh cac test mode

Dieu nay bien bai toan "cam giac motor nong / khung" thanh mot bai toan co so lieu:

- `Id` co dung dau khong
- `Iq` co bam ref khong
- voltage mode va current mode khac nhau cho nao

#### 3.3 Van de synchronization / frequency

GUI dong vai tro:

- doc monitor `Loop Freq`
- doc `Run Mode`, `Cal Status`, `Align Status`
- cho phep xac minh sau power-cycle he thong co that su quay lai trang thai san sang hay khong

No giup tach duoc 2 lop van de:

- firmware parser / parameter van song
- nhung nhip dieu khien thoi gian thuc co dang mat dong bo hay khong

### 4. Vai tro cua Trend, Scope va Snapshot trong brainstorming flow

#### 4.1 Trend Charts

Trend charts phu hop de nhin:

- xu huong cham hon
- quan he giua `Cmd` va `Act`
- behavior cua speed loop va position loop theo thoi gian

Day la cong cu quan trong de thay:

- overshoot
- rung quanh diem dung
- speed command co bi outer loop "quang roi" hay khong

#### 4.2 Scope / Trace

Scope phu hop de nhin:

- hien tuong nhanh
- current / voltage o tan so cao
- response trong tung chu ky test

No dac biet huu ich khi can soi:

- `Id` / `Iq`
- `Vd` / `Vq`
- waveform trong tuning mode

#### 4.3 Snapshot / Log

Snapshot va log giup:

- dong bang trang thai hien tai de phan tich
- luu bang chung cho moi buoc debug
- doi chieu parameter, mode, va feedback o mot thoi diem cu the

Trong luan van, day la nhom cong cu giup "ke lai cuoc dieu tra" ro rang nhat.

### 5. GUI va tinh lap lai cua thuc nghiem

Mot diem rat quan trong trong qua trinh phat trien la:

- neu mot bug khong lap lai duoc, thi rat kho loai tru

GUI giai quyet bai toan nay bang cach:

- dong nhat hoa nut bam thanh command co ten ro rang
- cho phep luon lap lai cung mot chuoi thao tac
- giu parameter va monitor trong cung mot man hinh

Vi vay, GUI khong chi phuc vu demo, ma phuc vu:

- repeatability
- observability
- isolation

Day chinh la 3 thu quyet dinh chat luong cua mot qua trinh debug servo drive.

### 6. Tong ket vai tro cua GUI trong luan van

Neu firmware la noi "ra quyet dinh dieu khien", thi GUI la noi:

- dat cau hoi dung
- tao bai test dung
- va doc ket qua dung

Do do, trong bo tai lieu luan van, file GUI khong nen chi dung lai o muc "command nao gui command gi", ma can duoc doc cung voi firmware flow de thay ro:

> Chung ta da xay dung mot he FOC co kha nang thao tac, quan sat, va debug co he thong, thay vi chi don thuan lam cho motor quay.

---

## 1. Tong quan luong

```text
User bam nut tren GUI
-> GUI tao frame protocol
-> Serial/USB CDC gui xuong driver
-> usbd_cdc_if.c nhan packet USB
-> USB_ReceiveData() day byte vao ring buffer
-> while(1) goi USB_ProcessData()
-> USB_ProcessData() tach frame + check CRC + xu ly command
-> firmware tra ACK ngay lap tuc
-> firmware co the set co/state/de ISR xu ly tiep
-> while(1) goi USB_TransmitData() de day monitor/read-data/error len PC
-> GUI parse frame nhan duoc va update man hinh
```

Noi ngan gon:

- `USB_ReceiveData()` chi co nhiem vu nhan byte thuan.
- `USB_ProcessData()` moi la noi "hieu lenh".
- `USB_TransmitData()` la noi gui du lieu response/update len PC.
- Vong control that su van nam trong ISR, khong nam trong `while(1)`.

---

## 2. Cac diem vao chinh trong firmware

### 2.1 USB receive callback

Khi PC gui du lieu xuong, USB CDC callback goi:

- [usbd_cdc_if.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Src/usbd_cdc_if.c):271

```c
USB_ReceiveData(UserRxBufferFS, *Len);
```

### 2.2 Byte duoc day vao ring buffer

- [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):662

```c
void USB_ReceiveData(uint8_t *data, uint32_t SizeOfData)
{
    for(uint32_t i=0; i<SizeOfData; i++){
        circBufPush(&rawCircBuffer, data[i]);
    }
}
```

Y nghia:

- Ham nay khong phan tich command
- khong check CRC
- khong doi state machine
- chi luu byte vao buffer thuan

### 2.3 Parser xu ly frame trong while(1)

- [main.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Src/main.c):204
- [main.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Src/main.c):205

```c
USB_ProcessData(&USB_Comm);
USB_TransmitData(&USB_Comm);
```

Ban chat:

- `USB_ProcessData()` doc ring buffer, rap frame, check CRC, dispatch command
- `USB_TransmitData()` gui monitor/read data/error packet len GUI

---

## 3. Protocol frame

Protocol cua project nay la frame nhi phan, khong phai text.

### 3.1 Frame command PC -> Driver

```text
STX | SYN | SIZE | CMD | DATA... | CRC | ETX
02    16    xx     xx    ...       xx    03
```

Trong do:

- `STX = 0x02`
- `SYN = 0x16`
- `ETX = 0x03`
- `SIZE = 1 + do dai DATA`
- `CMD = ma lenh`
- `CRC = tong checksum theo rule rieng cua project`

CRC cua project:

- khong cong byte `uCode`
- chi cong `SIZE + payload`
- payload o day la `CMD + DATA`

Ham tinh CRC:

- [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):121
- [USBCommunication.cpp](C:/Users/Admin/Desktop/FINAL_PROJECT/software/USBCommunication.cpp):236

### 3.2 ACK Driver -> PC

ACK co dang:

```text
STX | ACK | SIZE | CMD | DATA... | CRC | ETX
02    F0    xx     xx    ...       xx    03
```

Y nghia:

- ACK giu nguyen `SIZE`, `CMD`, `DATA`, `CRC`
- chi doi byte thu 2 tu `0x16` thanh `0xF0`

Ham tao ACK ben firmware:

- [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):101

Vi du:

- GUI gui `Servo ON`: `02 16 01 01 02 03`
- Driver ACK: `02 F0 01 01 02 03`

### 3.3 Frame update Driver -> PC

Khi driver gui monitor, read parameter, trace, error... no van gui kieu:

```text
STX | SYN | SIZE | SUBCMD | PAYLOAD... | CRC | ETX
```

Luc nay:

- byte `CMD/SUBCMD` la ma data update
- vi du `CMD_MONITOR_DATA = 0x35`
- GUI nhan frame, thay `code == 0x16` va `payload[0] == 0x35` thi parse theo monitor format

Code parse o GUI:

- [protocol.py](C:/Users/Admin/Desktop/FINAL_PROJECT/GUI/driver_gui_pyqt/protocol.py):224
- [main.py](C:/Users/Admin/Desktop/FINAL_PROJECT/GUI/driver_gui_pyqt/main.py):476

---

## 4. Frame format trong source

### 4.1 App cu Qt C++

Ham tao command frame:

- [USBApplication.cpp](C:/Users/Admin/Desktop/FINAL_PROJECT/software/USBApplication.cpp):6

Khung tao frame:

```c
STX
SYN
SIZE = uDataLength + 1
CMD
DATA...
CRC
ETX
```

### 4.2 Firmware STM32

Ham gui data update len PC:

- [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):136

Ham parser command tu PC:

- [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):669

---

## 5. Cac nut tren GUI Python hien tai gui lenh gi

GUI Python hien tai nam o:

- [main.py](C:/Users/Admin/Desktop/FINAL_PROJECT/GUI/driver_gui_pyqt/main.py)
- [protocol.py](C:/Users/Admin/Desktop/FINAL_PROJECT/GUI/driver_gui_pyqt/protocol.py)

Phan khai bao quick buttons:

- [main.py](C:/Users/Admin/Desktop/FINAL_PROJECT/GUI/driver_gui_pyqt/main.py):152
- [main.py](C:/Users/Admin/Desktop/FINAL_PROJECT/GUI/driver_gui_pyqt/main.py):257

### 5.1 Servo ON

- Nut GUI: `Servo ON`
- Command: `CMD_SERVO_ON = 0x01`
- Payload: khong co
- Frame gui:

```text
02 16 01 01 02 03
```

- ACK mong doi:

```text
02 F0 01 01 02 03
```

- Firmware handler:
  - [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):732

Xu ly:

- tra ACK ngay
- `STM_NextState(&StateMachine, OFFSET_CALIB);`

Y nghia:

- bam nut nay khong co nghia la PWM chay ngay lap tuc
- no chi chuyen state machine sang `OFFSET_CALIB`
- den cac chu ky ISR tiep theo moi vao `START`, roi `RUN`

### 5.2 Servo OFF

- Nut GUI: `Servo OFF`
- Command: `CMD_SERVO_OFF = 0x02`
- Payload: rong
- Frame gui:

```text
02 16 01 02 03 03
```

- ACK:

```text
02 F0 01 02 03 03
```

- Firmware handler:
  - [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):738

Xu ly:

- tra ACK
- `STM_NextState(&StateMachine, STOP);`

### 5.3 Refresh Monitor

- Nut GUI: `Refresh Monitor`
- Command: `CMD_UPDATE_MONITOR = 0x1C`
- Payload: rong
- Frame gui:

```text
02 16 01 1C 1D 03
```

- ACK:

```text
02 F0 01 1C 1D 03
```

- Firmware handler:
  - [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):842

Xu ly:

- tra ACK
- set `USB_Comm.ReadMotionMonitorData = 1`
- sau do `USB_TransmitData()` se gui `CMD_MONITOR_DATA = 0x35` len PC

### 5.4 ACK Fault

- Nut GUI: `ACK Fault`
- Command: `CMD_ACK_FAULT = 0x29`
- Payload: rong
- Frame gui:

```text
02 16 01 29 2A 03
```

- ACK:

```text
02 F0 01 29 2A 03
```

- Firmware handler:
  - [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):1181

Xu ly:

- tra ACK
- `STM_FaultAcknowledged(&StateMachine);`

### 5.5 Write To Flash

- Nut GUI: `Write To Flash`
- Command: `CMD_WRITE_TO_FLASH = 0x2B`
- Payload: rong
- Frame gui:

```text
02 16 01 2B 2C 03
```

- ACK:

```text
02 F0 01 2B 2C 03
```

- Firmware handler:
  - [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):1173

Xu ly:

- tra ACK
- ghi `DriverParameter[]` va `MotorParameter[]` xuong flash

### 5.6 Read Driver Params

- Nut GUI: `Read Driver Params`
- Command: `CMD_READ_DRIVER = 0x25`
- Payload: `tong so phan tu can doc`
- GUI hien tai gui `16` phan tu driver

Frame gui:

```text
02 16 02 25 10 37 03
```

Giai thich:

- `SIZE = 0x02` vi payload frame la `CMD + 1 byte length`
- `CMD = 0x25`
- `DATA[0] = 0x10` tuong ung 16 phan tu

- Firmware handler:
  - [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):1121

Xu ly:

- tra ACK
- set:
  - `USB_Comm.ReadDriverParameter = 1`
  - `USB_Comm.TotalLength = rxFrame.strData[0]`
  - `USB_Comm.PriorityFlag = 2`

Sau do `USB_TransmitData()` gui tung block `CMD_READ_DRIVER_DATA = 0x32`

Moi block co dang:

```text
SUBCMD(0x32) + [index + float] + [index + float] + ... + "\r\n"
```

Moi phan tu:

- 1 byte `index`
- 4 byte `float`

### 5.7 Read Motor Params

- Nut GUI: `Read Motor Params`
- Command: `CMD_READ_MOTOR = 0x26`
- Payload: `0x20` tuong ung 32 phan tu

Frame gui:

```text
02 16 02 26 20 48 03
```

- Firmware handler:
  - [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):1130

Xu ly:

- tra ACK
- set:
  - `USB_Comm.ReadMotorParameter = 1`
  - `USB_Comm.TotalLength = rxFrame.strData[0]`
  - `USB_Comm.PriorityFlag = 2`

Sau do `USB_TransmitData()` gui tung block `CMD_READ_MOTOR_DATA = 0x33`

### 5.8 Write Driver Params

- Nut GUI: `Write Driver Params`
- Command: `CMD_WRITE_DRIVER = 0x27`
- GUI dong goi thanh payload gom 16 cap:

```text
[index0][float0][index1][float1]...[index15][float15]
```

Moi cap chiem:

- 1 byte index
- 4 byte float little-endian

Tong payload:

- `16 * 5 = 80 byte`
- `SIZE = 81 byte` vi cong them 1 byte `CMD`

- Firmware handler:
  - [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):1139

Xu ly:

- tra ACK
- loop 16 lan
- moi lan:
  - doc `index`
  - doc `float`
  - `DriverParameter[index] = data`
- goi `UpdateDriverParameter(DriverParameter);`

Pseudo payload:

```text
00 <float param0>
01 <float param1>
02 <float param2>
...
0F <float param15>
```

### 5.9 Write Motor Params

- Nut GUI: `Write Motor Params`
- Command: `CMD_WRITE_MOTOR = 0x28`
- Firmware moi frame chi xu ly dung 20 cap `[index][float]`

Vi MotorParameter co 32 phan tu, GUI Python phai chia thanh 2 chunk:

- chunk 1: index 0 -> 19
- chunk 2: index 20 -> 31, sau do pad lap lai phan tu cuoi cho du 20 cap

Code GUI:

- [main.py](C:/Users/Admin/Desktop/FINAL_PROJECT/GUI/driver_gui_pyqt/main.py):432
- [protocol.py](C:/Users/Admin/Desktop/FINAL_PROJECT/GUI/driver_gui_pyqt/protocol.py):347

Firmware handler:

- [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):1156

Xu ly:

- tra ACK
- loop 20 lan
- moi lan doc `index` + `float`
- `MotorParameter[index] = data`
- xong goi `UpdateMotorParameter(MotorParameter);`

Luu y quan trong:

- firmware hien tai khong doc "so phan tu thuc"
- no mac dinh moi frame `CMD_WRITE_MOTOR` phai co dung 20 cap
- do do GUI phai chia chunk dung 20 cap

### 5.10 Send Raw

- Nut GUI: `Send Raw`
- Cho phep ban chon `CMD` va tu nhap payload hex
- GUI se tu dong dong goi thanh frame protocol day du

Vi du:

- chon `CMD_UPDATE_MONITOR = 0x1C`
- de payload rong
- GUI se gui:

```text
02 16 01 1C 1D 03
```

Neu nhap payload hex:

```text
10 00 00 80 3F
```

thi payload thuc se la 5 byte do, GUI se tu tinh `SIZE`, `CRC`, `ETX`

---

## 6. Driver tra gi ve cho GUI

### 6.1 ACK frame

Driver ACK ngay sau khi nhan va validate frame command.

Noi tao ACK:

- [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):734
- [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):740
- [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):844
- va cac nhanh xu ly khac tuong tu

### 6.2 Monitor data

Khi `CMD_UPDATE_MONITOR` duoc nhan:

- firmware set `USB_Comm.ReadMotionMonitorData = 1`
- den luot `USB_TransmitData()` no gui `CMD_MONITOR_DATA = 0x35`

Noi dong goi monitor:

- [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):621

Payload monitor hien tai:

1. `EnableRun` - 1 byte
2. `Vdc * 0.1f` - float
3. `Temperature` - float
4. `CmdSpeed` - float
5. `ActSpeed` - float
6. `SpeedError` - float
7. `CmdPos` - float
8. `ActPos` - float
9. `PosErr` - float
10. `IqRef` - float
11. `MotorPower` - int16
12. `FaultOccurred` - uint16

GUI Python parse:

- [protocol.py](C:/Users/Admin/Desktop/FINAL_PROJECT/GUI/driver_gui_pyqt/protocol.py):263

Luu y:

- firmware gui `Vdc * 0.1`
- GUI nhan ve roi nhan lai `*10`

### 6.3 Read driver / motor response

Driver gui:

- `CMD_READ_DRIVER_DATA = 0x32`
- `CMD_READ_MOTOR_DATA = 0x33`

Moi chunk:

- `[subcommand][index+float][index+float]...["\\r\\n"]`

GUI parse:

- [protocol.py](C:/Users/Admin/Desktop/FINAL_PROJECT/GUI/driver_gui_pyqt/protocol.py):335
- [main.py](C:/Users/Admin/Desktop/FINAL_PROJECT/GUI/driver_gui_pyqt/main.py):521

### 6.4 Error packet

Khi co fault:

- `USB_TransmitData()` gui `MTR_CODE_ERROR = 0xE1`

Noi dong goi:

- [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c):250

GUI parse:

- [protocol.py](C:/Users/Admin/Desktop/FINAL_PROJECT/GUI/driver_gui_pyqt/protocol.py):295

---

## 7. Cach dung GUI de test thuc te

### 7.1 Khi giao tiep that voi driver

Driver STM32 dung `USB CDC`.

Tren PC no se hien thanh mot `COM port`.

Vay luong that se la:

```text
GUI Python
-> COMx
-> USB cable
-> STM32 USB CDC
-> firmware
```

Ban khong can Hercules khi giao tiep that.

### 7.2 Khi test bang com0com

Neu test gia lap:

- GUI mo `COM7`
- Hercules mo `COM8`

Dieu nay hop le neu `COM7 <-> COM8` la mot cap noi voi nhau.

Nhung can hieu:

- GUI dong vai PC software
- Hercules dong vai driver gia
- Hercules se KHONG tu dong ACK

Nen khi ban bam:

- `Servo ON`

GUI gui:

```text
02 16 01 01 02 03
```

Neu ben `COM8` khong gui ACK nguoc lai, GUI se bao timeout.

ACK thu cong can gui tu Hercules:

```text
02 F0 01 01 02 03
```

Mot so lenh de test nhanh:

- `Servo ON`:

```text
02 16 01 01 02 03
```

- `Servo OFF`:

```text
02 16 01 02 03 03
```

- `ACK Fault`:

```text
02 16 01 29 2A 03
```

- `Refresh Monitor`:

```text
02 16 01 1C 1D 03
```

ACK tuong ung:

- chi doi byte thu 2 tu `16` thanh `F0`

Vi du:

```text
02 F0 01 1C 1D 03
```

Luu y:

- protocol la binary
- neu Hercules dang o che do ASCII thi nhieu byte trong nhu "khong co gi"
- nen de che do `HEX`

---

## 8. Flow chi tiet: bam nut -> firmware xu ly

### 8.1 Flow cua Servo ON

```text
User bam "Servo ON"
-> GUI goi _enqueue_command(CMD_SERVO_ON)
-> GUI tao frame 02 16 01 01 02 03
-> Driver nhan USB packet
-> USB_ReceiveData() push byte vao ring buffer
-> while(1) goi USB_ProcessData()
-> USB_ProcessData() bat duoc frame, check CRC dung
-> match CMD_SERVO_ON
-> tao ACK 02 F0 01 01 02 03
-> WriteComm() gui ACK len PC
-> STM_NextState(&StateMachine, OFFSET_CALIB)
-> ISR o cac chu ky sau chay OFFSET_CALIB -> START -> RUN
```

### 8.2 Flow cua Refresh Monitor

```text
User bam "Refresh Monitor"
-> GUI gui 02 16 01 1C 1D 03
-> Driver ACK 02 F0 01 1C 1D 03
-> USB_ProcessData() set USB_Comm.ReadMotionMonitorData = 1
-> while(1) goi USB_TransmitData()
-> USB_TransmitData() dong goi frame CMD_MONITOR_DATA = 0x35
-> GUI nhan frame, parse payload monitor
-> update cac label tren man hinh
```

### 8.3 Flow cua Read Driver Params

```text
User bam "Read Driver Params"
-> GUI gui CMD_READ_DRIVER voi payload 0x10
-> Driver ACK
-> USB_ProcessData() set ReadDriverParameter = 1, TotalLength = 16
-> USB_TransmitData() gui chunk CMD_READ_DRIVER_DATA
-> GUI parse moi chunk [index + float]
-> table Driver Parameters duoc fill len
```

### 8.4 Flow cua Write Motor Params

```text
User sua bang parameter
-> bam "Write Motor Params"
-> GUI cat thanh cac chunk 20 cap [index][float]
-> moi chunk gui CMD_WRITE_MOTOR
-> Driver ACK tung chunk
-> USB_ProcessData() ghi vao MotorParameter[index]
-> UpdateMotorParameter(MotorParameter)
-> neu muon luu flash thi bam them "Write To Flash"
```

---

## 9. Map command nhanh

Bang nay chi liet ke nhung command chinh lien quan GUI co ban hien tai.

| GUI action | Command | Hex | Payload | Firmware xu ly |
|---|---:|---:|---|---|
| Servo ON | `CMD_SERVO_ON` | `0x01` | rong | chuyen `OFFSET_CALIB` |
| Servo OFF | `CMD_SERVO_OFF` | `0x02` | rong | chuyen `STOP` |
| Refresh Monitor | `CMD_UPDATE_MONITOR` | `0x1C` | rong | set `ReadMotionMonitorData = 1` |
| ACK Fault | `CMD_ACK_FAULT` | `0x29` | rong | `STM_FaultAcknowledged()` |
| Write To Flash | `CMD_WRITE_TO_FLASH` | `0x2B` | rong | ghi flash |
| Read Driver Params | `CMD_READ_DRIVER` | `0x25` | 1 byte so luong | bat dau gui `0x32` |
| Read Motor Params | `CMD_READ_MOTOR` | `0x26` | 1 byte so luong | bat dau gui `0x33` |
| Write Driver Params | `CMD_WRITE_DRIVER` | `0x27` | 16 cap index-float | cap nhat `DriverParameter[]` |
| Write Motor Params | `CMD_WRITE_MOTOR` | `0x28` | 20 cap index-float | cap nhat `MotorParameter[]` |

---

## 10. Ban chat cua `USB_ReceiveData()` va `USB_ProcessData()`

Day la diem rat quan trong de buoc tiep theo minh va ban thiet ke lai cho sach.

### 10.1 `USB_ReceiveData()`

Ban chat dung nhat nen la:

- chi nhan byte tu USB interrupt/callback
- chi push vao ring buffer
- tuyet doi khong xu ly logic command o day

Code hien tai dang dung voi triet ly nay, va do la hop ly.

### 10.2 `USB_ProcessData()`

Ban chat nen tach thanh 3 lop:

1. `Frame parser`
   - tim `STX`
   - doc `SIZE`
   - doi den du frame
   - check `ETX`
   - check `CRC`

2. `Command decoder`
   - lay `CMD`
   - tach `DATA`
   - validate do dai

3. `Command handler`
   - `HandleServoOn()`
   - `HandleServoOff()`
   - `HandleReadDriver()`
   - `HandleWriteDriver()`
   - `HandleAckFault()`
   - ...

Hien tai project dang de ca 3 viec trong mot ham lon `USB_ProcessData()`.

Dieu nay van chay duoc, nhung:

- kho debug
- kho mo rong
- kho check sai payload
- kho unit test tung command

---

## 11. Huong thiet ke ham USB receive / command handler o buoc tiep theo

Muc tieu buoc tiep theo nen la:

```text
USB_ReceiveData()
-> push ring buffer

USB_TryParseFrame()
-> neu du frame thi tra ve frame hop le

USB_DispatchCommand(frame)
-> switch(cmd)
-> goi tung handler rieng

USB_HandleServoOn()
USB_HandleServoOff()
USB_HandleReadDriver()
USB_HandleWriteDriver()
USB_HandleReadMotor()
USB_HandleWriteMotor()
USB_HandleAckFault()
USB_HandleMonitorRequest()
```

Neu lam theo huong nay, ta se duoc:

- code de doc hon
- de them command moi
- de validate payload length
- de tach phan giao tiep va phan control/state

### 11.1 Goi y validate payload length

Moi command nen co rule ro rang:

- `CMD_SERVO_ON`: data length phai bang 0
- `CMD_SERVO_OFF`: data length phai bang 0
- `CMD_UPDATE_MONITOR`: data length phai bang 0
- `CMD_ACK_FAULT`: data length phai bang 0
- `CMD_READ_DRIVER`: data length phai bang 1
- `CMD_READ_MOTOR`: data length phai bang 1
- `CMD_WRITE_DRIVER`: data length phai bang `16 * 5 = 80`
- `CMD_WRITE_MOTOR`: data length phai bang `20 * 5 = 100`

Neu sai:

- bo frame
- hoac tra `ACK_ERROR`

### 11.2 Goi y tach frame struct

Struct command frame nen ro hon:

```c
typedef struct
{
    uint8_t code;      // SYN hoac ACK
    uint8_t size;      // CMD + DATA
    uint8_t cmd;       // command
    uint8_t data[250];
    uint8_t crc;
} UsbCommandFrame_t;
```

Nhu vay handler de doc hon rat nhieu so voi cach dung `strCommand[0]` + `strData`.

---

## 12. Ket luan ngan

Neu ban can nho nhanh, hay nho 4 y nay:

1. GUI bam nut la gui frame binary `02 16 SIZE CMD DATA CRC 03`
2. Driver nhan byte qua `USB_ReceiveData()`, nhung chi xu ly that su trong `USB_ProcessData()`
3. Driver ACK bang cach doi byte thu 2 thanh `F0`
4. Nhung lenh nhu `Servo ON` chi set state/cac co; control that su van chay trong ISR

---

## 13. File tham chieu chinh

- [USBComunication.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.c)
- [USBComunication.h](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Library/USBComunication.h)
- [main.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Src/main.c)
- [usbd_cdc_if.c](C:/Users/Admin/Desktop/FINAL_PROJECT/ASD04_RevE_LV2_20260119/ASD04_RevE_LV2/Src/usbd_cdc_if.c)
- [USBCommunication.cpp](C:/Users/Admin/Desktop/FINAL_PROJECT/software/USBCommunication.cpp)
- [USBCommunication.h](C:/Users/Admin/Desktop/FINAL_PROJECT/software/USBCommunication.h)
- [USBApplication.cpp](C:/Users/Admin/Desktop/FINAL_PROJECT/software/USBApplication.cpp)
- [main.py](C:/Users/Admin/Desktop/FINAL_PROJECT/GUI/driver_gui_pyqt/main.py)
- [protocol.py](C:/Users/Admin/Desktop/FINAL_PROJECT/GUI/driver_gui_pyqt/protocol.py)
