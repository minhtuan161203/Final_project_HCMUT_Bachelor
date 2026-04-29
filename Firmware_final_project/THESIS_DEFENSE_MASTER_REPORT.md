# ASD04 PMSM FOC Driver - Master Technical Report for Thesis Defense

## 1. Muc tieu cua tai lieu

Tai lieu nay la ban tong hop "master report" cho phan bao ve luan van, tap trung vao hanh trinh phat trien:

- servo PMSM dung Field-Oriented Control (FOC)
- bo commissioning / auto-tuning dien
- nhan dang co hoc `J` va `B`
- xu ly phi tuyen nghich luu, chong coupling, va giai quyet xung dot giua cac vong lap

Tai lieu nay duoc tong hop tu 3 nguon "ground truth":

1. `PROJECT_FLOW_VI.md`
2. lich su debug va quyet dinh ky thuat trong qua trinh trao doi
3. source code hien tai trong repo

Tai lieu nay KHONG co y "viet dep" mot cach marketing. Muc tieu la:

- noi dung phai defend duoc
- moi ket luan phai co logic ky thuat
- neu implementation hien tai khac voi "vision" ly tuong thi phai noi thang

## 2. Executive Summary

Ket qua cuoi cung cua de tai khong chi la "motor quay duoc", ma la mot chuoi nang cap ky thuat co tinh he thong:

- dong bo current loop theo heartbeat tu FPGA
- xac lap lai he quy chieu dien va chieu torque de FOC chay dung truc
- dua bo auto-tune dien `Rs -> Ls -> PolePairs/Flux`
- bo sung Uerror characterization de bu dead-time va inverter non-linearity
- dua decoupling vao vong current de giu `Id ~= 0` khi tang toc / dao chieu
- mo rong auto-tune sang nhan dang co hoc `J -> B`
- lam sach monitor / protocol / GUI de theo doi va apply duoc ket qua

Ve ban chat, day la hanh trinh di tu:

```text
"co model ly thuyet"
-> "co firmware chay duoc"
-> "co debug infrastructure"
-> "co bo commissioning on dinh"
-> "co bo auto-tune co the defend duoc"
```

## 3. Luu y quan trong ve pham vi va tinh trung thuc hoc thuat

Co 3 diem can noi ro ngay tu dau:

1. Repo hien tai KHONG tach rieng `current_loop.c` hay `foc_math.h`.
   Logic tuong duong dang nam chu yeu trong:
   - `Src/main.c`
   - `MDK-ARM/Library/PIDcontrol.c`
   - `MDK-ARM/Library/vector_transfs.c`

2. Cau chuyen `Uerror LUT` trong defense co the duoc trinh bay theo huong "LUT-based pre-compensation".
   Tuy nhien, branch code hien tai dang:
   - survey toi da `257` diem (`UERROR_SWEEP_MAX_POINTS`)
   - luu runtime LUT `33` diem (`UERROR_LUT_MAX_POINTS`)
   Nghia la firmware hien tai da chot o huong LUT, nhung ban implementation active la LUT nen / compact, khong phai 256-point runtime LUT day du.

3. "delta factor" theo tinh than InstaSPIN la mot guideline thiet ke bandwidth separation.
   Branch code hien tai khong dat mot bien ten la `delta`, nhung y tuong nay van duoc the hien qua:
   - current bandwidth mac dinh `200 Hz`
   - speed bandwidth mac dinh `20 Hz`
   - position bandwidth mac dinh `5 Hz`
   trong `motor_autotune.c`

## 4. Detailed Timeline - tu do thong so tho den he commissioning hoan chinh

### T0 - Bring-up va dong bo he thong

Ban dau bai toan khong chi la control, ma la architecture:

- STM32 khong tu chay doc lap
- he thong duoc dong bo theo heartbeat tu FPGA
- current, encoder, Vdc, nhiet do va PWM phai cung mot mien thoi gian

Neu lop dong bo nay sai, moi cong thuc control phia tren deu mat y nghia.

### T1 - Xu ly mau thuan he toa do va quy uoc chieu

Hien tuong som nhat:

- dong vong kin thi giat
- `Iq` nhin co ve hop ly nhung chieu quay sai
- bom `Vd` ma rotor van "nhuc nhich"

Ket luan:

- day khong phai loi co khi don thuan
- day la dau hieu he quy chieu dien / torque sign chua dung

Buoc nay rat quan trong vi neu he toa do sai, moi phep do `Rs`, `Ls`, `Flux`, `J`, `B` sau do deu co nguy co sai theo.

### T2 - Giai quyet "4x Rs error mystery"

Hien tuong:

- firmware do `Rs ~= 13.57 Ohm`
- VOM do ngoai thuc te chi khoang `3.4 Ohm`
- sai lech xap xi `4x`

Gia thuyet dung:

- current feedback dang bi scale sai
- neu dong do nho hon thuc te khoang 4 lan, thi `Rs = V/I` se bi doi len khoang 4 lan

Day la mot bai hoc rat thuc chien:

```text
FOC co the van "chay"
nhung parameter identification va gain synthesis se sai nghiem trong
neu current scaling sai.
```

### T3 - Lam dung auto-tune dien

Sau khi he toa do va current scaling duoc lam dung, auto-tune dien moi tro nen dang tin:

- `Rs` bang stepping `Id` tren rotor khoa
- `Ls` bang kich thich sin tren truc D rotor khoa
- `PolePairs` bang open-loop electrical turns / mechanical turns
- `Flux` va `Ke` bang open-loop V/F + toc do trung binh

### T4 - Xu ly phi tuyen nghich luu bang Uerror characterization

Hien tuong:

- dead-time lam bien dang dien ap hieu dung
- gan zero current / zero crossing co ripple `Id`
- `Rs` va current response o toc do thap khong con "sach"

Huong di chot:

- khong co gang "lam nhu nghich luu ly tuong"
- do truc tiep sai lech `Uerror(Iphase)`
- dua no thanh LUT de runtime pre-compensation

### T5 - Tach vong lap va xu ly xung dot speed / position

Hien tuong:

- speed loop rat em, nhung position loop van overshoot
- cảm giac controller "mau thuan noi bo"

Phan tich:

- bandwidth position va speed dang qua gan nhau
- position integrator co the windup
- position PI + speed PI neu dat tham se de sinh overshoot

### T6 - Xu ly advanced FOC dynamics bang decoupling

Hien tuong:

- khi tang toc cao hoac dao chieu, `Id` bi lech khoi 0
- co luc `Id` doi dau du speed ref van hop ly

Ket luan:

- day la dq cross-coupling that
- khong phai "noise ngam nhien" hay "PI loi"

### T7 - Mo rong sang nhan dang co hoc `J/B`

Hien tuong:

- flow auto-tune dung o `82%` khi do `J`
- sau khi sua `J`, flow lai dung o `92%` khi do `B`

Nguyen nhan:

- detector zero-cross cu qua mong manh
- LPF tre pha + deadband hep `+-1 rpm` lam detector "mu" tai luc rotor dao chieu nhanh

Giai phap:

- doi detector sang kieu hysteresis / Schmitt trigger
- tu do moi chot duoc cua so tich phan / hoi quy hop le

## 5. Control Architecture - khung xuong song cua toan bo he thong

Kien truc control cua driver la cascaded control:

```text
Position loop -> Speed loop -> Current loop -> PWM
```

Trong repo hien tai:

- current PI va current-loop orchestration: `Src/main.c`
- PI primitive: `MDK-ARM/Library/PIDcontrol.c`
- Clarke/Park/InvPark/InvClarke: `MDK-ARM/Library/vector_transfs.c`
- parameter update / current feedback conversion: `MDK-ARM/Library/Parameter.c`
- protocol/monitor/autotune telemetry: `MDK-ARM/Library/USBComunication.c`

## 6. Hardware Calibration & Scaling - phan "4x Error" cua Rs

### 6.1 Hien tuong

`Rs` do bang firmware cao hon thuc te khoang 4 lan:

```text
Rs_firmware ~= 13.57 Ohm
Rs_VOM ~= 3.4 Ohm
```

### 6.2 Phan tich ky thuat

Cong thuc `Rs` trong autotune la:

```text
Rs ~= DeltaV / DeltaI
```

Neu `I` bi do nho hon thuc te khoang 4 lan, `Rs` se bi phong dai khoang 4 lan.

Trong repo, current duoc doi tu raw FPGA/ADC sang ampere tai:

- `Parameter.c`
- `main.c`

Cong thuc scale hien tai:

```c
fIabc = -(raw_current - offset) / 65536 * INPUT_RANGE_I
```

Do do, moi sai so trong:

- offset calibration
- `INPUT_RANGE_I`
- analog gain
- ADC/FPGA conversion chain

se di truc tiep vao `Rs`, `Ls`, current PI, va ca fault threshold.

### 6.3 Root cause va cach chot khi defend

Root cause dung nhat de bao ve:

- khong phai cong thuc `Rs` sai
- khong phai PI sai
- ma la current sensing scaling / ADC conversion chain chua khop voi thuc te phan cung

Cach tra loi ngan gon:

> Sai so `Rs x4` khong phai la loi identification algorithm ma la loi metrology. Khi current feedback bi scale sai, firmware van co the dieu khien duoc mot phan, nhung parameter estimation va gain synthesis se sai. Vi vay buoc chinh dau tien la recalibrate current scale de dong ampere trong firmware tro ve dung voi dong thuc te.

### 6.4 Code neo thuc te

- `MDK-ARM/Library/Parameter.c`: `fIabc` scale tu raw feedback
- `Src/main.c`: cung cong thuc current scaling duoc dung lai trong runtime path
- `MDK-ARM/Library/define.h`: `INPUT_RANGE_I`, `Resolution16bits`, `OFFSET`
- `MDK-ARM/Library/motor_autotune.c`: `MotorAutoTune_ProcessRs()`

## 7. Electrical Auto-tuning - nen tang theo huong TI / InstaSPIN

### 7.1 Current PI foundation

Cong thuc current PI dang dung trong repo:

```text
Kp_current = Ls * w_bw
Ki_current = Rs * w_bw
```

Day la cong thuc cung tinh than InstaSPIN-FOC:

```text
Kp = L * BW
Ki = R * BW
```

Trong source:

- `motor_autotune.c` cap nhat current gain tu `measured_Rs`, `measured_Ls`, `current_bandwidth_hz`

### 7.2 Loop-separation theo tinh than delta factor

Concept "delta factor" dung trong defense de noi ve separation giua cac vong:

- current loop phai nhanh nhat
- speed loop cham hon current loop mot boi so an toan
- position loop cham hon speed loop

Branch code hien tai the hien y tuong nay qua mac dinh:

```text
BW_current = 200 Hz
BW_speed   = 20 Hz
BW_pos     = 5 Hz
```

Nghia la:

- current:speed = 10:1
- speed:position = 4:1

Neu hoi ve "delta", co the tra loi:

> Trong implementation hien tai, chung toi khong dat rieng mot bien ten `delta`, nhung y tuong delta duoc ma hoa thanh bandwidth separation va default tuning ratio. Dieu quan trong khong phai ten bien, ma la current loop phai nhanh hon speed loop mot khoang cach du lon de outer loop khong nhin thay noi dong hoc noi tai cua inner loop.

### 7.3 Rs, Ls, Flux

#### Rs

Rotor khoa, q-axis bi isolate:

```text
Rs ~= DeltaVd / DeltaId
```

#### Ls

Kich thich sin tren truc D rotor khoa:

```text
|Z| = V_peak / I_peak
Ls = sqrt(|Z|^2 - Rs^2) / omega
```

#### PolePairs + Flux

Open-loop V/F:

```text
PolePairs ~= electrical_turns / mechanical_turns
Flux ~= V_peak / omega_e
Ke = Flux
```

Repo hien tai con co "best-effort Rs compensation" trong flux estimation:

```text
E_rms ~= sqrt(V_rms^2 - (I_rms * Rs)^2)
```

Neu gia tri bu tro nen phi vat ly, firmware fallback ve estimate dien ap cu thay vi abort.

## 8. Uerror va Inverter Non-linearity

### 8.1 Ban chat van de

Dead-time, diode drop, saturation va sai lech nghich luu lam cho:

- dien ap command `Vabc` khong con bang dien ap hieu dung tai pha
- gan zero current, error doi dau theo chieu dong
- current loop de thay ripple va "notch" tai zero crossing

He qua la:

- do `Rs` o vung dong nho de sai
- current loop gan zero-speed / zero-current kem min
- flux estimation o toc do thap kem dang tin

### 8.2 Quy trinh characterization

Repo hien tai da dua vao mot flow rieng:

1. lock theta theo runtime FOC frame da validate
2. sweep current doi xung tren truc D
3. lay trung binh `Id`, `Iphase`, `Vphase`, `Vdc`, nhiet do
4. trich xuat `Uerror(Iphase)`
5. luu thanh LUT runtime

### 8.3 Su that can noi ro khi defend

Can noi rat trung thuc:

- branch code hien tai survey toi da `257` diem
- nhung runtime LUT hien luu `33` diem normalize trong flash

Vi vay, cach noi tot nhat la:

> Huong ky thuat da chot la LUT-based inverter pre-compensation. Ban firmware hien tai dung survey mat do cao de thu du lieu, sau do nen thanh LUT runtime gon hon de toi uu flash, protocol va chi phi tinh toan. Ban chat giai phap van la bu phi tuyen nghich luu theo bang tra.

### 8.4 Runtime compensation

Firmware noi suy LUT theo dong pha, nhan voi `Vdc`, sau do tru common-mode:

```text
comp_phase = LUT(Iphase) * Vdc
Vphase_cmd += comp_phase - common_mode
```

Day la mot diem rat manh khi defend:

- compensation khong pha tong ba pha
- van giu tong common-mode hop ly
- van dat tren mien `Vabc`, tuc la sat voi inverter that

### 8.5 Code neo thuc te

- `USBComunication.h`
  - `UERROR_SWEEP_MAX_POINTS = 257`
  - `UERROR_LUT_MAX_POINTS = 33`
- `main.c`
  - `BuildUerrorSweepTargets()`
  - `StartUerrorCharacterization()`
  - `LookupUerrorLutNorm()`
  - `ApplyUerrorCompensationToPhaseVoltages()`
  - `SaveUerrorLutToFlash()`
- `USBComunication.c`
  - start/stop/apply/save command handlers
  - chunk streaming du lieu survey ve GUI

## 9. Advanced FOC Dynamics - Decoupling

### 9.1 Hien tuong

O toc do cao hoac khi dao chieu:

- `Id` khong giu duoc quanh 0
- co the doi dau
- PI current co ve "khong giong bench test rotor khoa"

### 9.2 Phan tich vat ly

Day la hien tuong coupling giua d-axis va q-axis:

```text
Vd ~ -omega_e * Lq * Iq
Vq ~  omega_e * (Ld * Id + psi_f)
```

Neu khong bu, PI d-axis phai tu ganh mot thanh phan van toc phu thuoc `omega_e`, nen khi speed tang len thi `Id` bi keo lech.

### 9.3 Implementation hien tai

Repo hien tai da them current-loop decoupling theo convention da validate tren project:

```text
vd +=  omega_e * L * iq
vq += -omega_e * (L * id + psi_f)
```

Luu y quan trong:

- project nay dang dung sign convention ma `Iq` am moi tao torque duong tang toc
- vi vay dau cua decoupling phai theo convention thuc nghiem cua project, khong the copy xi textbook

Day la mot cau tra loi rat manh neu bi hoi van:

> Decoupling khong chi la cong thuc, ma la cong thuc phai nhung dung vao quy uoc dau cua he thong. Neu quy uoc torque sign trong firmware khac textbook, thi dau cua feed-forward term cung phai theo quy uoc firmware.

### 9.4 Code neo thuc te

- `main.c`
  - `ApplyCurrentLoopDecoupling()`
  - `RunCurrentLoopForTheta()`
  - runtime FOC loop gan `gIdPi`, `gIqPi`

## 10. Cascaded Loop Conflict - Speed em nhung Position overshoot

### 10.1 Hien tuong

- speed response rat em
- position response lai overshoot
- co luc position loop "day" speed loop qua tay

### 10.2 Phan tich

Day la bai toan 2 tang PI noi nhau:

- position PI sinh `speed_reference_rpm`
- speed PI sinh `Iq_ref`

Neu:

- position `Ki` qua lon
- speed bandwidth va position bandwidth qua gan nhau
- hoac khong co deadband / hold hop ly quanh zero

thi position loop se nap tich phan va day speed loop vao saturate hoac overshoot.

### 10.3 Giai phap defend duoc

Co 3 huong giai thich / xu ly:

1. tach bandwidth ro hon
2. giam `Ki` cua position loop
3. trong case can robust hon, dung `P-Position + PI-Speed`

Repo hien tai da ho tro huong nay vi:

- `GetConfiguredPositionIntegralGain()` clamp `Ki >= 0`
- neu muon bo `Ki`, co the set `POSITION_I_GAIN = 0`
- code da co deadband hold va reset PI quanh zero de han che chatter

Nghia la ban co the defend theo huong:

> Kien truc toi uu cho vi tri khong bat buoc luc nao cung phai la PI-PI. Neu co khi tai va encoder tao ra noise / deadband ro, thi `P-Position + PI-Speed` thuong de bao dam stability hon va van dat du tracking. Branch hien tai giu position PI de tong quat, nhung cho phep ep `Ki = 0` de quay ve cau truc P-Position khi can.

### 10.4 Code neo thuc te

- `main.c`
  - `GetConfiguredPositionIntegralGain()`
  - `UpdatePositionSetpointVelocityRpm()`
  - deadband / release deadband
  - `ApplySpeedRampLimit()`
  - runtime do `gPositionPi` va `gSpeedPi`

## 11. Mechanical Identification - J va B

### 11.1 Tai sao can offline mechanical identification

`J` va `B` la hai tham so rat nhay voi:

- tai gan vao rotor
- ma sat co khi
- encoder noise
- current noise
- timing skew giua `Iq`, `omega`, `alpha`

Vi vay huong chot dung la:

- khong cap nhat online luc drive dang van hanh thong thuong
- chay mot commissioning step offline
- excitation duoc thiet ke truoc
- co filter va integration window ro rang

### 11.2 Uoc luong J

Firmware sau stage `Flux` da co:

```text
Kt = 1.5 * PolePairs * Flux
Te = Kt * Iq
```

Sau do kich `Iq` hinh sin doi xung va tinh:

```text
J = integral(Te * alpha dt) / integral(alpha^2 dt)
```

Ly do chon cach nay:

- profile doi xung giup triet tieu anh huong `T_L` tot hon step response
- khong phai vi phan truc tiep `omega`
- dung tich phan nen de chong nhieu hon

### 11.3 "82 percent stuck" - deep dive

Tai sao progress dung dung o `82%`?

Vì:

- `82%` la moc bat dau stage `J`
- progress chi tang khi firmware chot duoc `mechanical_window_count`
- trong ban dau, detector doi dau can:
  - toc do vuot deadband rat hep
  - sign doi
  - tich `previous_speed * current_speed <= 0`

Van de la:

- toc do da qua LPF nen co tre pha
- rotor dao chieu nhanh qua zero
- sample co the di qua vung `+-1 rpm` ma detector khong "arm" kip
- ket qua la graph thi thay cat zero, nhung firmware lai khong dem crossing

### 11.4 Giai phap - Hysteresis / Schmitt trigger

Detector moi su dung hai nguong:

- `release` threshold
- `switch` threshold

Y tuong:

1. rotor phai di vao gan zero de arm detector
2. sau do phai vuot nguong o phia ben kia moi tinh la mot crossing hop le

Day la cach giai thich rat manh:

> Loi khong nam o cong thuc `J`, ma nam o viec firmware co nhan dien dung "cua so tich phan hop le" hay khong. Schmitt-trigger khien detector ben vung hon voi LPF phase lag va noise quanh zero-speed.

### 11.5 Uoc luong B

Sau khi co `J`, firmware dung mo hinh:

```text
Te = J * alpha + B * omega + T_L
```

Ban ly tuong trong luan van co the dan den cong thuc dung `dTe/dt`, nhung tren MCU that no rat nhay voi nhieu.

Vi vay branch hien tai chon huong:

- giu excitation doi xung
- tich luy du lieu qua nhieu nua chu ky
- giai hoi quy cho `B` va `T_L`

### 11.6 "92 percent stuck" - vi sao B bi y chang J

Pattern giong het:

- `92%` la moc vao state `B`
- zero-cross detector cu cung mu voi LPF + deadband
- ket qua regression window khong bao gio dong lai duoc

Va giai phap cung la:

- doi detector sang hysteresis cho stage `B`

### 11.7 Gia tri defend duoc cua cach tiep can nay

Ban co the tra loi rat ro:

- day la integral-based identification
- `J` bam sat huong luan van
- `B` dung implementation practical hon de chiu nhieu tot hon
- toan bo flow la offline autotune, khong phai online adaptive estimation

## 12. Tai sao chon Integral Method thay vi Step Response

| Phuong phap | Uu diem | Nhuoc diem |
| --- | --- | --- |
| Integral method voi excitation doi xung | giam anh huong `T_L`, dung tich phan nen de chong nhieu, tach `J` ro hon | can excitation profile, can detector zero-cross tot |
| Step response don gian | de lam, de giai thich, nhanh | `T_L` va `B` tron vao nhau, `J` de sai neu co friction / load |
| Vi phan truc tiep tu encoder | mo hinh "thang" | qua nhay voi nhieu, rat de vo tren MCU |

Ket luan defend:

> Step response hop de bench nhanh, nhung de lay `J` co y nghia cho servo PMSM co load that thi integral method ro rang dang tin hon, du doi hoi implementation ky hon o phan timing, LPF va detector zero-cross.

## 13. Tai sao Offline autotune thay vi Online RLS / RLS-like estimation

| Huong | Uu diem | Nhuoc diem |
| --- | --- | --- |
| Offline autotune | de kiem soat profile kich thich, de loc / trung binh, an toan hon cho commissioning | khong cap nhat khi tai thay doi trong luc chay |
| Online RLS / adaptive ID | co the theo doi tham so thay doi theo tai | nhay voi excitation condition, matrix update, drift, va do phuc tap firmware |

Ket luan defend:

> Muc tieu cua de tai la tao driver co commissioning suite on dinh tren STM32/FPGA platform, khong phai mot bo adaptive control nghien cuu. Vi vay offline autotune la diem can bang hop ly nhat giua do chinh xac, do phuc tap va do tin cay.

## 14. Code Reference Map - map ly thuyet sang source code thuc te

### 14.1 Neu hoi dong hoi "current_loop.c va foc_math.h nam o dau?"

Cau tra loi dung voi repo hien tai:

- `current_loop.c` tuong duong:
  - `Src/main.c`
  - `MDK-ARM/Library/PIDcontrol.c`
- `foc_math.h` tuong duong:
  - `MDK-ARM/Library/vector_transfs.h`
  - `MDK-ARM/Library/vector_transfs.c`
  - mot so helper control trong `Src/main.c`

### 14.2 Bang map nhanh

| Chu de | File chinh | Y nghia |
| --- | --- | --- |
| Current scaling / ADC -> Ampere | `MDK-ARM/Library/Parameter.c`, `MDK-ARM/Library/define.h` | goc cua bai toan `Rs x4` |
| Runtime FOC orchestration | `Src/main.c` | speed loop, position loop, current PI, decoupling, PWM |
| PI primitive | `MDK-ARM/Library/PIDcontrol.c` | block PI dung lai giua current/speed/position |
| Transform alpha-beta / dq | `MDK-ARM/Library/vector_transfs.c` | Clarke/Park/InvPark/InvClarke |
| Electrical auto-tune | `MDK-ARM/Library/motor_autotune.c` | `Rs`, `Ls`, `PolePairs`, `Flux`, `J`, `B` |
| Uerror characterization & runtime LUT | `Src/main.c`, `MDK-ARM/Library/USBComunication.c/h` | sweep, LUT, flash, protocol |
| Telemetry monitor / GUI link | `MDK-ARM/Library/USBComunication.c` | stream `Rs`, `Ls`, `Flux`, `J`, `B`, Uerror chunk |
| Parameter storage | `MDK-ARM/Library/define.h`, `Src/main.c` | storage IDs, flash load/save, unit scaling |

### 14.3 Reference chi tiet

- `Parameter.c`
  - current scaling: `fIabc = ... * INPUT_RANGE_I`
- `main.c`
  - `LookupUerrorLutNorm()`
  - `ApplyUerrorCompensationToPhaseVoltages()`
  - `ApplyCurrentLoopDecoupling()`
  - runtime `gPositionPi`, `gSpeedPi`, `gIdPi`, `gIqPi`
  - `RunCurrentLoopForTheta()`
  - `RunMotorAutoTuneLoop()`
- `motor_autotune.c`
  - `MotorAutoTune_ProcessRs()`
  - `MotorAutoTune_ProcessFlux()`
  - `MotorAutoTune_ProcessJ()`
  - `MotorAutoTune_ProcessB()`
  - `MotorAutoTune_Finish()`
- `USBComunication.h`
  - `UERROR_SWEEP_MAX_POINTS`
  - `UERROR_LUT_MAX_POINTS`
  - `UerrorCharacterization_t`
- `define.h`
  - `MOTOR_PARAMETER_COUNT`
  - `MOTOR_ROTOR_INERTIA`
  - `MOTOR_VISCOUS_FRICTION`
  - `POSITION_I_GAIN` alias

## 15. HAO PROFESSOR DEFENSE SECTION - trap questions and best answers

### Q1. Tai sao `Rs` firmware sai toi 4 lan?

**Best answer**

Khong phai do cong thuc `Rs` sai, ma do current metrology sai. `Rs` duoc tinh tu `V/I`, nen chi can current scale sai thi `Rs` sai tuong ung. Sau khi recalibrate current sensing scaling va ADC conversion chain, `Rs` firmware quay ve khop voi VOM, va khi do current PI / autotune moi co y nghia vat ly.

### Q2. Tai sao khong dung step response cho nhanh, can gi phuc tap voi integral method?

**Best answer**

Step response khong tach dep `J` khoi `B` va `T_L`. Integral method voi excitation doi xung cho phep triệt tieu anh huong tai tinh tot hon va chong nhieu tot hon, du implementation tren MCU kho hon.

### Q3. Tai sao `J` dung o 82%? Algorithm sai hay motor sai?

**Best answer**

Cong thuc `J` khong sai. Phan sai nam o detector zero-cross cua firmware. LPF + deadband hep lam detector bo qua crossing khi rotor dao chieu nhanh, nen firmware khong bao gio dong duoc cua so tich phan. Sau khi chuyen sang detector hysteresis, progress moi di tiep.

### Q4. Tai sao `B` khong dung thang cong thuc co `dTe/dt` nhu luan van?

**Best answer**

Vi `dTe/dt` va dao ham bac cao cua toc do qua nhay voi nhieu tren encoder/current thuc. Branch firmware dung hoi quy tren du lieu da loc va cac cua so nua chu ky, van bam dung mo hinh vat ly `Te = J*alpha + B*omega + T_L` nhung chiu nhieu tot hon.

### Q5. Tai sao `Id` lai lech khi tang toc, du `Id_ref = 0`?

**Best answer**

Do dq cross-coupling, cu the thanh phan speed-dependent nhay vao `Vd` va `Vq`. Khi speed tang, neu khong decouple thi d-axis PI phai tu ganh term do, nen `Id` lech. Feed-forward decoupling da duoc them vao current loop de bo term nay.

### Q6. Tai sao speed loop em ma position loop van overshoot?

**Best answer**

Vi outer position PI va inner speed PI dang conflict. Nguyen nhan thuong la position `Ki` qua lon hoac bandwidth separation chua du. Huong chot la giam `Ki`, tang separation, hoac quay ve `P-Position + PI-Speed`.

### Q7. Delta factor co that su duoc implement chua?

**Best answer**

Neu hoi theo ten bien thi khong co mot bien `delta` rieng trong source. Neu hoi theo tinh than thiet ke thi co: current loop, speed loop, va position loop duoc tach bandwidth ro rang, va cac default `200/20/5 Hz` phan anh dung triet ly do.

### Q8. Tai sao khong chay online RLS de co `J/B` cap nhat lien tuc?

**Best answer**

Vi muc tieu cua de tai la driver commissioning on dinh tren phan cung that. Online RLS doi hoi excitation persistency, matrix update ben vung, va co nguy co drift. Offline autotune la diem can bang hop ly hon giua do chinh xac va do tin cay.

### Q9. Uerror LUT cua ban la 256-point hay 33-point?

**Best answer**

Huong giai phap da chot la LUT-based pre-compensation. Trong branch firmware hien tai, survey du lieu co mat do cao hon, con LUT runtime duoc nen lai thanh 33 diem de tiet kiem flash, protocol, va chi phi tinh toan. Neu can LUT day hon, kien truc hien tai hoan toan mo rong duoc.

### Q10. Tai sao current PI lai phai dung `Kp = L*BW`, `Ki = R*BW`?

**Best answer**

Vi inner current loop cua PMSM sau bien doi dq duoc xap xi boi mo hinh `L di/dt + R i = v`. Dat cuc theo bandwidth mong muon dan den dang tinh gain nhu tren. Day la cach dat gain chuan, nhat quan voi triet ly InstaSPIN.

## 16. Conclusion - cau chuyen nen chot khi bao ve

Neu can tom tat toan bo hanh trinh trong mot doan ngan:

> De tai nay da di qua dung cac lop kho nhat cua mot servo drive that: tu metrology, he quy chieu dien, dong bo thoi gian thuc, phi tuyen nghich luu, dq cross-coupling, cho den nhan dang co hoc. Gia tri cua cong trinh khong nam o mot cong thuc rieng le, ma nam o viec bien cac cong thuc do thanh mot firmware commissioning suite co the chay duoc, debug duoc, va defend duoc tren phan cung thuc.

Neu can tom tat trong 3 y de slide ket:

1. Dung model la chua du; phai dung current scale, dung frame, dung timing.
2. Muon autotune dang tin tren MCU, phai uu tien metrology, filtering, va window detection.
3. Gia tri ky thuat lon nhat cua de tai la bien mot bo FOC ly thuyet thanh mot driver co the commissioning va bao ve hoc thuat duoc.
