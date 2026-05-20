# Firmware Q&A Walkthrough

## 1. Muc tieu cua tai lieu

Tai lieu nay tong hop lai noi dung trao doi ky thuat ve project servo drive hien tai, theo van phong lich su, ro y, va de dung lai cho:

- hoc source code
- chuan bi bao ve
- tra loi khi thay hoi tai sao firmware va hardware duoc thiet ke nhu hien tai

Tai lieu nay khong co muc tieu "viet dep" theo kieu marketing. Muc tieu la:

- noi dung phai dung voi source active
- moi ket luan phai noi ro no den tu code nao
- neu co cho nao la convention hay suy luan ky thuat thi phai noi ro

---

## 2. Buc tranh lon cua project

Neu nhin theo goc do dieu khien tu dong, day khong chi la mot project STM32 thong thuong, ma la mot chuoi servo drive gom:

- FPGA: dong bo thoi gian va doc feedback
- STM32F407: tinh toan dieu khien thoi gian thuc
- TIM8: tao PWM 3 pha cho nghich luu
- GUI: commissioning, monitor, test, tuning

Tu duy thiet ke chinh cua project la:

```text
FPGA giu nhip va feedback
-> STM32 tinh FOC
-> TIM8 phat PWM
-> inverter tao dien ap 3 pha cho motor
```

Hay noi ngan hon:

```text
do luong dong bo
-> uoc luong trang thai
-> position/speed/current control
-> FOC
-> PWM
-> nghich luu
```

---

## 3. Cau hinh hardware chinh va y nghia

### 3.1 MCU va clock

MCU la `STM32F407ZET6`, package `LQFP144`, cau hinh tu file `.ioc`.

Clock dung `HSE + PLL` de dat:

- `SYSCLK = 168 MHz`
- `APB1 = 42 MHz`
- `APB2 = 84 MHz`

Ly do:

- du suc cho current loop, speed loop, USB, FSMC
- co `TIM8` la advanced timer
- co `FSMC` de noi FPGA theo kieu memory-mapped
- co `USB OTG FS` de giao tiep GUI

### 3.2 TIM8 la timer cong suat chinh

`TIM8` duoc dung vi no ho tro:

- 3 cap PWM bo sung `CHx/CHxN`
- deadtime
- center-aligned PWM
- rat hop cho bo nghich luu 3 pha

### 3.3 Vi sao dung center-aligned

Timer duoc dat `TIM_COUNTERMODE_CENTERALIGNED3`.

Y nghia:

- xung PWM doi xung quanh giua chu ky
- switching deu hon
- EMI va meo hai thuong de chiu hon edge-aligned
- thuan loi cho current sampling va FOC

Neu edge-aligned thi xung bi don ve mot mep chu ky. Neu center-aligned thi xung nam can doi quanh tam chu ky.

### 3.4 Vi sao can deadtime

Deadtime duoc chen de tranh high-side va low-side cua cung mot pha cung dan mot luc, gay `shoot-through`.

Day la yeu cau an toan cong suat, khong phai chi la yeu cau "tinh dep" cua PWM.

### 3.5 Vi sao TIM8 dong bo theo ETR ngoai

TIM8 duoc slave reset theo `ETR` ngoai. Dieu nay cho thay PWM carrier duoc dong bo theo nhip do FPGA cap.

Tu duy o day la:

- khong de timer cong suat "tu chay"
- khong de vong dieu khien va feedback bi lech nhip
- toan he thong chay theo mot moc thoi gian chung

### 3.6 Vi sao dung FSMC noi FPGA

FPGA duoc map qua `FSMC`, khong phai `SPI`.

Ly do:

- current loop can doc feedback nhanh va deu
- `FSMC` cho phep doc register nhu doc bo nho
- giam protocol overhead va jitter so voi SPI

Noi cach khac:

```text
SPI = co overhead giao tiep
FSMC = memory-mapped register access nhanh hon cho real-time control
```

### 3.7 Vi sao do dong theo 2-shunt

Firmware doc `Iu`, `Iv`, roi suy ra:

```text
Iw = -(Iu + Iv)
```

Ly do:

- giam phan cung do dong
- van du thong tin cho FOC vi tong 3 dong pha ly tuong bang 0

---

## 4. Firmware chay nhu the nao

Trong project nay, `while(1)` khong phai vong dieu khien chinh.

Co 2 lop:

- `while(1)`: xu ly USB, command, monitor, tham so
- `HAL_GPIO_EXTI_Callback()`: vong dieu khien thoi gian thuc

Noi ngan gon:

```text
while(1) = supervisory / HMI
EXTI ISR = regulation / servo loop
```

`HAL_GPIO_EXTI_Callback()` duoc kich boi `iMeasureIsr`, chinh la "nhip tim that" cua servo.

---

## 5. Luong firmware tu boot den run

Trinh tu tong quat:

1. `main()` init peripheral, GPIO, timer, USB, FSMC
2. nap tham so motor va driver
3. enable front-end do
4. cau hinh parser encoder / FPGA side
5. `while(1)` cho va xu ly command
6. moi khi co `iMeasureIsr`, firmware:
   - doc feedback
   - kiem tra fault
   - cap nhat speed / theta
   - chay FOC hoac mode test
   - cap nhat PWM

---

## 6. `UpdateMeasuredSpeedAndTheta()` dang lam gi

Ham nay bien feedback encoder thanh:

- toc do co `fActSpeed`
- goc dien `fTheta`

Y tuong:

```text
DeltaPos -> speed
EncSingleTurn + offset -> mechanical angle -> electrical angle
```

### 6.1 Tinh toc do

Cong thuc co ban:

```text
speed_rpm = DeltaPos * f_isr * 60 / encoder_resolution
```

Day la dao ham roi rac cua vi tri.

### 6.2 Vi sao can filter alpha thay doi theo toc do

O toc do thap:

- `DeltaPos` rat nho
- nhieu luong tu hoa
- toc do do duoc rat nhay

Nen code dung `alpha` nho hon o toc do thap, roi tang dan den nguong khoang `120 rpm`.

Y nghia:

- toc do thap: loc manh hon
- toc do cao: bam nhanh hon

### 6.3 Tinh goc co va goc dien

Doan code quan trong:

```c
if (MotorParameter[MOTOR_CURRENT_CTRL_DIRECTION] == 0.0f)
{
	position_single_turn = (float)Parameter.EncSingleTurn;
}
else
{
	position_single_turn = encoder_resolution - (float)Parameter.EncSingleTurn;
}

position_single_turn += (float)Parameter.Offset_Enc;
position_single_turn = fmodf(position_single_turn, encoder_resolution);
if (position_single_turn < 0.0f)
{
	position_single_turn += encoder_resolution;
}

mechanical_angle = (1.0f - (position_single_turn / encoder_resolution)) * (2.0f * PI);
electrical_angle = mechanical_angle * (float)Parameter.u8PolePair;
Parameter.fTheta = WrapAngle(electrical_angle);
```

Y nghia:

- xu ly chieu tang count cua encoder
- cong offset can chinh
- doi count thanh goc co
- nhan so doi cuc de ra goc dien

---

## 7. Tai sao co dong `1.0f - (...)` trong tinh `mechanical_angle`

Dong code:

```c
mechanical_angle = (1.0f - (position_single_turn / encoder_resolution)) * (2.0f * PI);
```

co nghia la:

```text
theta = 2pi - theta_raw
```

No dang dao chieu goc.

### 7.1 Tai sao can dao chieu

Vi thu vien vector transform cua project da ngam dinh mot quy uoc chieu duong rieng.

Trong `vector_transfs.c`, Clarke va Park cho thay:

```c
alpha = A;
beta  = (A + 2B)/sqrt(3);

d = alpha*cos(theta) + beta*sin(theta);
q = beta*cos(theta) - alpha*sin(theta);
```

Tu cong thuc nay co the suy ra:

- `+alpha` trung voi pha `A/U` duong
- trong he `alpha-beta`, goc duong tang theo chieu CCW cua toan hoc
- `+q` nam truoc `+d` 90 do theo chieu do

Neu encoder raw tang theo chieu nguoc lai, thi phai doi sang:

```text
theta = 2pi - theta_raw
```

### 7.2 Lam sao biet no phu hop voi lib

Khong the chi nhin comment ma khang dinh. Phai xac nhan bang:

- cong thuc Park / Inverse Park trong lib
- current feedback mapping
- encoder alignment
- rotating-theta tests
- current feedback map test

Neu convention dung thi:

- lenh `Iq` tao torque sach
- `Id` khong bi ro lon
- rotating theta test quay muot
- position / speed / torque cung quy uoc dau

---

## 8. Thu vien vector transform dang duoc dung

Sau khi don source, project hien con dung 4 ham:

### 8.1 `tFRClarke_ab2albe`

```c
ptFRClarke->fAl = ptFRClarke->fA;
ptFRClarke->fBe = (ptFRClarke->fA + (2.0f * ptFRClarke->fB)) / SQRT_3_F;
```

Y nghia:

- tu 2 dong pha do duoc `A`, `B`
- doi sang he `alpha-beta`

Vi project do 2 dong, nen day la reduced Clarke.

### 8.2 `tFPark_albe2dq`

```c
ptFPark->fD = (ptFPark->fAl * ptFPark->fCosAng) + (ptFPark->fBe * ptFPark->fSinAng);
ptFPark->fQ = (ptFPark->fBe * ptFPark->fCosAng) - (ptFPark->fAl * ptFPark->fSinAng);
```

Y nghia:

- quay he `alpha-beta` ve he `dq`
- bien dong AC 3 pha thanh 2 thanh phan gan nhu DC

Trong do:

- `Id` lien quan den tu thong
- `Iq` lien quan den torque

### 8.3 `tIPark_dq2albe`

```c
ptIPark->fAl = (ptIPark->fD * ptIPark->fCosAng) - (ptIPark->fQ * ptIPark->fSinAng);
ptIPark->fBe = (ptIPark->fQ * ptIPark->fCosAng) + (ptIPark->fD * ptIPark->fSinAng);
```

Y nghia:

- doi nguoc `Vd/Vq` ve `alpha-beta`

### 8.4 `tIFClarke_albe2abc`

```c
ptIFClarke->fA = ptIFClarke->fAl;
ptIFClarke->fB = 0.5f * ((-ptIFClarke->fAl) + (SQRT_3_F * ptIFClarke->fBe));
ptIFClarke->fC = 0.5f * ((-ptIFClarke->fAl) - (SQRT_3_F * ptIFClarke->fBe));
```

Y nghia:

- doi nguoc tu he 2 truc `alpha-beta`
- thanh 3 dien ap pha that `A/B/C`

---

## 9. `RunFocLoop()` dang lam gi

`RunFocLoop()` la noi cac vong lap duoc noi lai thanh mot he servo hoan chinh.

Flow tong quat:

```text
chon control_theta
-> Iabc sang Id/Iq
-> tao Id_ref, Iq_ref
-> current PI sinh Vd, Vq
-> Vd/Vq doi nguoc thanh Vabc
-> Vabc doi thanh duty PWM
```

### 9.1 Chon `control_theta`

`control_theta` khong phai luc nao cung bang `Parameter.fTheta`.

No co the:

- dung `Parameter.fTheta`
- cong runtime frame offset
- bi ep trong alignment
- bi thay bang theta test trong cac mode diagnostic

Y nghia:

- `theta` do duoc va `theta` dua vao Park co the khac nhau
- day la cach project commissioning va debug convention cua FOC

### 9.2 Dong pha sang `Id/Iq`

Current loop phan tich:

```text
Iu, Iv
-> current mapping
-> Clarke
-> Park
-> Id, Iq
```

Neu `theta` sai:

- `Id` va `Iq` bi tron
- torque yeu
- motor rung, nong

### 9.3 Speed loop va position mode

Trong mode thuong:

- current loop chay moi ISR
- speed loop chia 2, vi du tu 16 kHz xuong 8 kHz

Trong position mode active hien tai:

- khong dung `gPositionPi.m_calc()`
- dung planner `PlanPositionSpeedReferenceRpm()`
- roi moi dua `speed_reference_rpm` cho `gSpeedPi`

### 9.4 Current PI

Sau khi co `Id_ref`, `Iq_ref`:

- PI truc `d` sinh `Vd`
- PI truc `q` sinh `Vq`

Sau do con co:

- decoupling
- voltage vector limiting

Cuoi cung moi doi ve `Vabc` va PWM.

---

## 10. Thu vien PI dang duoc dung

Sau khi don source, project hien chi con dung `PI`.

Ham:

- `tPI_calc()`
- `tPI_rst()`

### 10.1 Code cua `tPI_calc()`

```c
void tPI_calc(tPI *ptPI)
{
	float fPreOut;
	float fIin;
	float fIout;

	ptPI->fPout = ptPI->fIn * ptPI->fKp;

	fIin = ptPI->fIn * ptPI->fKi;
	fIout = ptPI->fIprevOut + (0.5f * ptPI->fDtSec * (fIin + ptPI->fIprevIn));
	ptPI->fIprevIn = fIin;

	fPreOut = ptPI->fPout + fIout;

	if (fPreOut > ptPI->fUpOutLim)
	{
		fPreOut = ptPI->fUpOutLim;
		fIout = fPreOut - ptPI->fPout;
	}
	if (fPreOut < ptPI->fLowOutLim)
	{
		fPreOut = ptPI->fLowOutLim;
		fIout = fPreOut - ptPI->fPout;
	}

	ptPI->fIout = fIout;
	ptPI->fIprevOut = fIout;
	ptPI->fOut = fPreOut;
}
```

### 10.2 Y nghia tung phan

Nhanh `P`:

```text
P = Kp * e
```

do:

```c
ptPI->fPout = ptPI->fIn * ptPI->fKp;
```

Nhanh `I`:

```text
I[k] = I[k-1] + 0.5 * dt * (Ki*e[k] + Ki*e[k-1])
```

do:

```c
fIout = ptPI->fIprevOut + (0.5f * ptPI->fDtSec * (fIin + ptPI->fIprevIn));
```

Day la tich phan hinh thang, khong phai Euler tien.

Tong output truoc saturation:

```text
u_pre = P + I
```

do:

```c
fPreOut = ptPI->fPout + fIout;
```

### 10.3 Anti-windup

Khi `u_pre` vuot limit:

```c
fPreOut = ptPI->fUpOutLim;
fIout = fPreOut - ptPI->fPout;
```

hoac:

```c
fPreOut = ptPI->fLowOutLim;
fIout = fPreOut - ptPI->fPout;
```

Y nghia:

- output da bi saturation
- nen tich phan khong duoc tiep tuc "phinh len vo ich"
- phai keo `Iout` ve de tong `P + I` bang dung muc saturation

Day la mot dang anti-windup.

### 10.4 `tPI_rst()`

```c
void tPI_rst(tPI *ptPI)
{
	ptPI->fIn = 0.0f;
	ptPI->fIout = 0.0f;
	ptPI->fIprevIn = 0.0f;
	ptPI->fIprevOut = 0.0f;
	ptPI->fOut = 0.0f;
	ptPI->fPout = 0.0f;
}
```

Ham nay xoa trang thai cua PI, dac biet la thanh phan tich phan, tranh viec doi mode xong bi "da" boi gia tri cu.

---

## 11. Cach firmware kich bo nghich luu hien tai

Repo active hien tai khong viet `SVPWM sector/T1/T2/T0` mot cach tuong minh.

No dang lam theo chuoi:

```text
FOC PI -> Vd/Vq
-> inverse Park
-> inverse Clarke
-> Vabc
-> duty_u / duty_v / duty_w
-> TIM8 phat PWM complementary
```

### 11.1 Doan code doi `Vabc` sang duty

```c
inv_vbus = 1.0f / Parameter.fVdc;
duty_u = 0.5f + (Parameter.fVabc[0] * inv_vbus);
duty_v = 0.5f + (Parameter.fVabc[1] * inv_vbus);
duty_w = 0.5f + (Parameter.fVabc[2] * inv_vbus);
```

Y nghia:

- `50%` la muc trung tinh
- pha nao can dien ap duong hon thi duty tang
- pha nao can dien ap am hon thi duty giam

### 11.2 TIM8 lam gi

TIM8 phat:

- `CH1/CH1N`
- `CH2/CH2N`
- `CH3/CH3N`

tuong ung 3 cap transistor tren / duoi cua bo nghich luu.

Noi ngan gon:

- firmware tinh 3 duty
- TIM8 lo 6 xung that

---

## 12. Position mode active hien tai dung planner, khong dung `gPositionPi`

Day la diem rat quan trong khi tune.

Trong nhanh position mode:

```c
if (sPositionDeadbandHoldActive != 0u)
{
	gPositionPi.m_rst(&gPositionPi);
	gSpeedPi.m_rst(&gSpeedPi);
	...
	speed_reference_rpm = 0.0f;
}
else
{
	gPositionPi.m_rst(&gPositionPi);
	...
	speed_reference_rpm = PlanPositionSpeedReferenceRpm(...);
}
```

Ta thay:

- `gPositionPi` duoc `m_rst()`
- khong co `gPositionPi.m_calc()`
- `speed_reference_rpm` duoc sinh boi planner

Vi vay, trong source active hien tai:

- tang `POSITION_P_GAIN`
- tang `POSITION_I_GAIN`

gan nhu khong thay doi duong dieu khien position mode runtime.

Nhung:

- `SPEED_P_GAIN`
- `SPEED_I_GAIN`
- `ACCELERATION_TIME`
- `DECELERATION_TIME`
- `MAXIMUM_SPEED`

lai co tac dong ro rang.

---

## 13. Planner dang lam gi

Ham chinh:

```c
static float PlanPositionSpeedReferenceRpm(
	float target_position_counts,
	float actual_position_counts,
	float current_command_rpm,
	float actual_speed_rpm,
	float encoder_resolution,
	float max_speed_rpm,
	float dt_sec)
```

Planner nay khong phai position PI co dien. No la mot bo sinh `speed_reference_rpm` online co xet:

- khoang cach con lai
- kha nang giam toc
- gioi han toc do
- ramp accel / decel

### 13.1 Tinh sai so vi tri

```c
position_error_counts = GetPositionControlErrorCounts(
	target_position_counts,
	actual_position_counts,
	encoder_resolution);
```

Neu mode la single-turn thi sai so duoc wrap ve huong ngan nhat trong 1 vong. Neu la multi-turn thi giu sai so thang.

### 13.2 Xac dinh chieu chay

```c
if (position_error_counts > 0.0f)
{
	desired_direction = -1.0f;
}
else if (position_error_counts < 0.0f)
{
	desired_direction = 1.0f;
}
else
{
	return 0.0f;
}
```

Project giu quy uoc dau cu:

- loi duong -> speed ref am

### 13.3 Doi sai so thanh quang duong con lai

```c
remaining_revolutions = fabsf(position_error_counts) / encoder_resolution;
```

Planner quy doi so count thanh so vong co con lai.

### 13.4 Tinh quang phanh can thiet

```c
planning_speed_rpm = fmaxf(fabsf(current_command_rpm), fabsf(actual_speed_rpm));
actual_speed_rps = planning_speed_rpm / 60.0f;
decel_rps2 = decel_rpm_per_sec / 60.0f;
braking_distance_rev = (actual_speed_rps * actual_speed_rps) / (2.0f * decel_rps2);
```

Day chinh la cong thuc co hoc:

```text
s = v^2 / (2a)
```

### 13.5 Neu gan dich thi ha tran toc do

```c
if (remaining_revolutions <= braking_distance_rev)
{
	braking_speed_limit_rpm = sqrtf(2.0f * decel_rps2 * remaining_revolutions) * 60.0f;
}
else
{
	braking_speed_limit_rpm = max_speed_rpm;
}
```

Y nghia:

- con xa: cho chay toi da
- den vung phai phanh: ha toc do theo quang con lai

### 13.6 Qua ramp limiter

Cuoi cung:

```c
target_speed_rpm = desired_direction * braking_speed_limit_rpm;

return ApplySpeedRampLimit(
	current_command_rpm,
	target_speed_rpm,
	max_speed_rpm,
	dt_sec);
```

Nghia la speed target van phai di qua gioi han accel / decel tung chu ky.

### 13.7 Ban chat cua planner

Planner hien tai gan nhat voi:

```text
online trapezoidal / braking-aware speed planner
```

No khong phai:

- position PI co dien
- hay full trajectory planner phuc tap

No lam theo y:

```text
con xa -> cho chay nhanh
gan dich -> phai biet luc nao can ham
ham xong -> dua speed ref cho speed PI
```

---

## 14. Auto-tuning dang lam gi

Auto-tune trong repo nay la mot bo `offline identification + gain synthesis`.

No khong phai adaptive control online, va cung khong phai mot module tach roi khoi FOC. Nguoc lai, no ke thua chinh:

- he quy chieu `dq` da duoc xac minh
- current feedback mapping da duoc chot
- `control_theta` runtime da duoc canh align

Noi cach khac:

```text
Auto-tune khong "tu song rieng"
ma muon do tham so dung thi phai dung chung frame voi FOC that
```

### 14.1 State machine autotune

Trong `motor_autotune.h`, cac state chinh la:

- `RS`
- `LS`
- `FLUX`
- `J`
- `B`
- `DONE`
- `ERROR`

Flow tong quat:

```text
Rs -> Ls -> PolePairs/Flux -> J -> B -> synthesize gains
```

### 14.2 Firmware moi chu ky autotune dang lam gi

Trong `RunMotorAutoTuneLoop()`:

```c
inputs.id_current_a = Parameter.fIdq[0];
inputs.iq_current_a = Parameter.fIdq[1];
inputs.vd_voltage_v = Parameter.fVdq[0];
inputs.vq_voltage_v = Parameter.fVdq[1];
inputs.phase_voltage_u_v = Parameter.fVabc[0];
inputs.phase_voltage_v_v = Parameter.fVabc[1];
inputs.phase_voltage_w_v = Parameter.fVabc[2];
inputs.phase_u_a = Parameter.fIabc[0];
inputs.phase_v_a = Parameter.fIabc[1];
inputs.phase_w_a = Parameter.fIabc[2];
inputs.bus_voltage_v = Parameter.fVdc;
inputs.electrical_theta_rad = electrical_theta;
inputs.mechanical_position_counts = Parameter.fPosition;
inputs.mechanical_speed_rpm = Parameter.fActSpeed;
```

Y nghia:

- firmware chup lai mot "snapshot" sach cua he thong
- dua snapshot do vao state machine autotune
- de autotune quyet dinh stage nay can:
  - dung current loop
  - hay dung voltage d truc tiep
  - hay dung open-loop V/F

Doan switch output mode trong `RunMotorAutoTuneLoop()` cho thay ro:

- `MOTOR_AUTOTUNE_OUTPUT_CURRENT_LOOP`
- `MOTOR_AUTOTUNE_OUTPUT_DIRECT_D_VOLTAGE`
- `MOTOR_AUTOTUNE_OUTPUT_OPEN_LOOP_VF`

Tuc la autotune khong chi "doc", ma con chon cach kich plant phu hop voi moi bai toan identification.

### 14.3 Stage `Rs`: do dien tro stator

Code cot loi:

```c
outputs->mode = MOTOR_AUTOTUNE_OUTPUT_CURRENT_LOOP;
outputs->id_ref_a = current_target;
outputs->iq_ref_a = 0.0f;
outputs->isolate_q_axis = 1u;
...
delta_i = high_i_avg - low_i_avg;
delta_v = high_v_avg - low_v_avg;

if (MotorAutoTune_Abs(delta_i) > MOTOR_AUTOTUNE_MIN_RS_CURRENT_A)
{
	handle->measured_Rs = MotorAutoTune_Abs(delta_v / delta_i);
}
```

Y tuong dieu khien:

- rotor dung yen
- giu `Iq = 0`
- chi bom `Id` tren truc `d`
- current loop giu dong o 2 muc:
  - `I_low`
  - `I_high`
- do trung binh `Vd` va `Id`
- suy ra:

```text
Rs ~= DeltaVd / DeltaId
```

Co so ly thuyet:

Trong rotor-lock, bo qua suc dien dong quay, mo hinh truc `d` gan dung:

```text
Vd = Rs * Id + Ld * d(Id)/dt
```

Khi da cho settle du lau, thanh phan dao ham gan nhu bang 0:

```text
Vd ~= Rs * Id
```

nen:

```text
Rs ~= Vd / Id
```

Dung `DeltaV / DeltaI` thay vi 1 diem tuyet doi giup giam lech do offset.

Uu diem:

- de hieu, de bao ve
- an toan hon vi chi kich tren truc `d`
- tach khoi torque `q-axis`

Nhuoc diem:

- rat nhay voi current scaling sai
- nhay voi offset current va sai so inverter gan zero
- neu dong qua nho thi `DeltaI` nho, ti so `V/I` se nhieu
- `Rs` cung thay doi theo nhiet do

### 14.4 Stage `Ls`: do cam stator

Code cot loi:

```c
outputs->mode = MOTOR_AUTOTUNE_OUTPUT_DIRECT_D_VOLTAGE;
commanded_vd = voltage_peak_v * sinf(omega * elapsed_s);
...
impedance_mag_ohm = voltage_peak_v / current_peak_a;
reactive_term_sq =
	(impedance_mag_ohm * impedance_mag_ohm) -
	(handle->measured_Rs * handle->measured_Rs);
...
handle->measured_Ls = sqrtf(reactive_term_sq) / omega;
```

Y tuong dieu khien:

- van giu rotor khoa va `q-axis` tach rieng
- thay vi dong current loop, stage nay ap mot `Vd` hinh sin truc tiep
- do dap ung dong `Id`
- tinh bien do tro khang:

```text
|Z| = V_peak / I_peak
```

voi mo hinh `RL`:

```text
|Z|^2 = Rs^2 + (omega Ls)^2
```

nen:

```text
Ls = sqrt(|Z|^2 - Rs^2) / omega
```

Co so ly thuyet:

Tren truc `d`, khi rotor khoa:

```text
Vd(s) / Id(s) = Rs + sLs
```

kich bang sin tan so `omega` se cho tro khang phuc:

```text
Z(jw) = Rs + jwLs
```

Firmware dang dung phan bien do cua `Z(jw)` de suy `Ls`.

Uu diem:

- kha sach vi do tren rotor khoa
- phan tach duoc `Rs` va `Xs = wL`
- de noi voi mo hinh RL kinh dien

Nhuoc diem:

- phu thuoc manh vao `Rs` do truoc do
- can chon tan so kich thich hop ly
- neu dong qua nho, peak current rat de nhieu
- `Ls` co the phu thuoc muc dong vi saturation

### 14.5 Stage `PolePairs` va `Flux`

Code cot loi:

```c
outputs->mode = MOTOR_AUTOTUNE_OUTPUT_OPEN_LOOP_VF;
outputs->vf_frequency_hz = test_frequency_hz;
outputs->vf_voltage_v = test_voltage_v;
handle->commanded_electrical_turns += MotorAutoTune_Abs(test_frequency_hz) * handle->dt_s;
mechanical_turns = MotorAutoTune_Abs(
	(inputs->mechanical_position_counts - handle->mechanical_position_start_counts) /
	encoder_resolution);
...
ratio = handle->commanded_electrical_turns / mechanical_turns;
handle->measured_PolePairs = floorf(ratio + 0.5f);
```

Y tuong:

- tao tu truong quay open-loop
- dem xem da "lenh" bao nhieu vong dien
- rotor that quay duoc bao nhieu vong co
- suy ra:

```text
PolePairs ~= electrical_turns / mechanical_turns
```

Sau do stage flux:

```c
phase_rms = sqrtf(handle->flux_voltage_sq_sum / (float)handle->flux_samples);
phase_current_rms = sqrtf(handle->flux_current_sq_sum / (float)handle->flux_samples);
resistive_drop_rms = phase_current_rms * handle->measured_Rs;
...
electrical_omega_rad_s = 2.0f * PI * average_rpm * handle->measured_PolePairs / 60.0f;
handle->measured_Flux = (phase_rms * 1.41421356f) / electrical_omega_rad_s;
handle->measured_Ke = handle->measured_Flux;
```

Co so ly thuyet:

Voi PMSM dang quay, bo qua mot so chi tiet phi ly tuong:

```text
E_peak ~= omega_e * Flux
```

nen:

```text
Flux ~= E_peak / omega_e
```

Firmware lay RMS dien ap pha, doi sang peak, roi chia cho toc do dien. No con co mot bu "best-effort" cho su sut ap tren `Rs`:

```text
E_rms ~= sqrt(V_rms^2 - (I_rms * Rs)^2)
```

Neu bu nay tro thanh phi vat ly do nhieu hoac model dien ap chua dep, code fallback ve estimate don gian hon thay vi abort.

Uu diem:

- khong can current loop qua phuc tap
- phu hop cho commissioning ban dau
- dat duoc `PolePairs` va `Flux` tu chinh chuyen dong that cua rotor

Nhuoc diem:

- phu thuoc vao rotor co theo kip tu truong open-loop khong
- kem dang tin o toc do qua thap
- nhay voi sai so `Vabc`, `Rs`, va nonlinearity cua inverter

### 14.6 Stage `J`: nhan dang moment quan tinh

Repo co 2 huong:

- `legacy` mode: kich dao dong co hoc doi xung
- `loaded` mode: threshold-based accel/decel

Mac dinh config la `legacy`.

Trong `legacy` mode, code cot loi la:

```c
handle->mechanical_window_numerator +=
	raw_torque_nm * handle->mechanical_accel_filtered_rad_s2 * handle->dt_s;
handle->mechanical_window_denominator +=
	handle->mechanical_accel_filtered_rad_s2 *
	handle->mechanical_accel_filtered_rad_s2 * handle->dt_s;
...
handle->measured_J = MotorAutoTune_Abs(
	handle->mechanical_window_numerator /
	handle->mechanical_window_denominator);
```

Co so ly thuyet:

Mo hinh co hoc don gian:

```text
Te = J * alpha + B * omega + Tload
```

Gan vung dao dong doi xung, neu xu ly cua so hop ly thi thanh phan tai tinh va ma sat co the duoc giam anh huong. Neu coi `Te ~= J * alpha` la thanh phan chinh trong cua so tinh, thi dat bai toan binh phuong toi thieu:

```text
min_J integral (Te - J*alpha)^2 dt
```

dao ham theo `J` va cho bang 0 se ra:

```text
J = integral(Te * alpha dt) / integral(alpha^2 dt)
```

Code dang thuc hien dung cong thuc do duoi dang tich luy roi rac.

Torque duoc doi tu `Iq` thong qua:

```text
Kt = 1.5 * pole_pairs * Flux
Te = Kt * Iq
```

Uu diem:

- dung tich phan nen it nhay nhieu hon dao ham truc tiep
- tach `J` ro hon step test don gian
- rat hop voi bai toan offline commissioning

Nhuoc diem:

- phu thuoc vao detector zero-cross va cua so chap nhan
- LPF speed / accel neu dat khong hop ly se gay tre pha
- neu `Flux` sai thi torque quy doi sang `Te` cung sai

### 14.7 Stage `B`: nhan dang ma sat nhot

Code cot loi:

```c
corrected_torque_nm =
	handle->mechanical_torque_sign *
	handle->mechanical_torque_constant_nm_per_a *
	inputs->iq_current_a;
regression_output = corrected_torque_nm -
	(handle->measured_J * handle->mechanical_accel_filtered_rad_s2);
...
handle->mechanical_regression_sw2 += current_speed_rad_s * current_speed_rad_s * handle->dt_s;
handle->mechanical_regression_sw += current_speed_rad_s * handle->dt_s;
handle->mechanical_regression_s11 += handle->dt_s;
handle->mechanical_regression_swy += current_speed_rad_s * regression_output * handle->dt_s;
handle->mechanical_regression_sy += regression_output * handle->dt_s;
...
handle->measured_B = (
	(handle->mechanical_regression_swy * handle->mechanical_regression_s11) -
	(handle->mechanical_regression_sy * handle->mechanical_regression_sw)) /
	determinant;
```

Co so ly thuyet:

Sau khi da co `J`, firmware xet:

```text
Te - J*alpha = B*omega + Tload
```

Dat:

```text
y = Te - J*alpha
```

thi bai toan tro thanh hoi quy tuyen tinh:

```text
y = B*omega + Tload
```

Code dang cong don cac tong:

- `sum(w^2)`
- `sum(w)`
- `sum(1)`
- `sum(wy)`
- `sum(y)`

roi giai nghiem least-squares 2 an:

- `B`
- `Tload`

Uu diem:

- practical hon cong thuc dung dao ham cao
- co kha nang tach `B` va tai tinh `Tload`
- phu hop voi du lieu encoder/current thuc te nhieu nhieu

Nhuoc diem:

- chat luong phu thuoc truc tiep vao `J` da do truoc do
- gia thiet ma sat nhot tuyen tinh co the khong dung hoan toan
- can cua so du lieu du rong va bien toc ro

### 14.8 Sau khi co tham so, firmware tinh bo dieu khien nhu the nao

Phan tong hop gain nam trong `MotorAutoTune_Finish()`.

#### 14.8.1 Current PI

Code:

```c
current_bw_rad_s = 2.0f * PI * handle->config.current_bandwidth_hz;
handle->tuned_current_kp = handle->measured_Ls * current_bw_rad_s;
handle->tuned_current_ki = handle->measured_Rs * current_bw_rad_s;
```

Co so ly thuyet:

Current loop sau bien doi `dq` duoc xap xi:

```text
L di/dt + R i = v
```

hay:

```text
Plant(s) = I(s)/V(s) = 1 / (Ls + R)
```

Dung PI:

```text
C(s) = Kp + Ki/s
```

Dat zero cua PI de "phu hop" voi cuc cua plant va chon bandwidth mong muon `omega_bw`, ta co dang thong dung:

```text
Kp = L * omega_bw
Ki = R * omega_bw
```

Y nghia:

- `Kp` di theo `L` vi no phai xu ly dong hoc nhanh cua dong
- `Ki` di theo `R` vi no xoa sai so tinh cua plant co ton tai thanh phan dien tro

Uu diem:

- rat chuan sach giao khoa cho current loop PMSM
- de giai thich, de tune theo bandwidth
- nhat quan voi triet ly current-loop first

Nhuoc diem:

- phu thuoc vao `Rs`, `Ls` da do dung
- bo qua phi tuyen, sat, decoupling chua hoan hao
- khi sampling / inverter / noise manh, bandwidth chon qua cao se de rung

#### 14.8.2 Speed PI

Code:

```c
torque_constant = 1.5f * pole_pairs * handle->measured_Flux;
handle->tuned_speed_kp = (rotor_inertia * speed_bw_rad_s) / torque_constant;
handle->tuned_speed_ki = handle->tuned_speed_kp * (speed_bw_rad_s * 0.25f);
```

Co so ly thuyet:

Neu coi inner current loop da nhanh va `Iq` bam tot, thi tu `Iq` den toc do co hoc co the xap xi:

```text
J * domega/dt = Kt * Iq - B*omega - Tload
```

Neu tam thoi bo qua `B` va `Tload` trong buoc dat gain thuan, plant tu `Iq` den `omega` la:

```text
Omega(s) / Iq(s) ~= Kt / (J s)
```

voi:

```text
Kt = 1.5 * pole_pairs * Flux
```

Neu chon PI cho plant kieu tich phan nay, dang `Kp ~ J*omega_bw/Kt` la rat hop ly:

```text
Kp_speed ~= J * omega_bw / Kt
```

Day chinh la cong thuc code dang dung.

Con `Ki` trong code:

```text
Ki_speed = Kp_speed * omega_bw * 0.25
```

khong phai dang dat cuc "sach giao khoa" duy nhat, ma la mot lua chon thuc dung, bao thu hon so voi `Ki ~ J*omega_bw^2/Kt`.

Co the hieu no nhu:

- van tang theo `omega_bw`
- van tang theo `J`
- van giam khi `Kt` lon
- nhung giam bot do "hung han" cua tich phan de tranh outer loop qua gac

Noi thang:

```text
Kp_speed = mo hinh ro rang
Ki_speed = heuristic practical / damping-friendly hon
```

Uu diem:

- phu hop thuc te hon khi `J`, `B`, `Flux` khong hoan hao
- de outer speed loop ben vung hon
- giam kha nang inner/outer loop xung dot

Nhuoc diem:

- khong phai cong thuc "toi uu ly tuong" cho moi plant
- neu muon response speed rat gac, co the phai fine-tune them

#### 14.8.3 Position PI

Code:

```c
position_output_scale = 60.0f / encoder_resolution_counts;
handle->tuned_position_kp = position_output_scale * (2.0f * position_bw_rad_s);
handle->tuned_position_ki = position_output_scale * (position_bw_rad_s * position_bw_rad_s);
```

Co so ly thuyet:

Position PI trong project nay duoc hieu la:

- input: sai so vi tri theo `counts`
- output: speed command theo `rpm`

Neu doi count sang rev qua he so:

```text
speed_output_scale = 60 / encoder_resolution
```

va chon dang chuan cua he bac hai:

```text
s^2 + 2*omega_n*s + omega_n^2
```

thi gain vi tri co the dat theo tinh than:

```text
Kp_pos ~ 2*omega_n
Ki_pos ~ omega_n^2
```

roi nhan them he so doi don vi `count -> rpm`.

Dieu rat quan trong:

Trong branch runtime active hien tai, position mode dang dung planner, khong dung `gPositionPi.m_calc()` tren duong chinh. Nghia la:

- autotune van tinh `POSITION_P_GAIN`, `POSITION_I_GAIN`
- van apply vao bang tham so
- nhung nhanh runtime hien tai gan nhu khong su dung truc tiep bo gain nay

### 14.9 Separation bandwidth va triet ly dieu khien

Default trong autotune:

```text
BW_current = 200 Hz
BW_speed   = 20 Hz
BW_pos     = 5 Hz
```

Day la mot cach ma hoa triet ly cascade control:

- inner current nhanh nhat
- speed cham hon current khoang 10 lan
- position cham hon speed khoang 4 lan

Y nghia theo co so dieu khien tu dong:

- outer loop nen "nhin" inner loop nhu mot plant da duoc on dinh hoa
- neu bandwidth gan nhau qua, cac vong lap se tranh chap nhau
- tach ro bandwidth giup tuning de va ben vung hon

### 14.10 Uu va nhuoc cua huong offline autotune hien tai

Uu diem:

- excitation duoc kiem soat
- an toan hon cho commissioning
- de log, de trung binh, de phat hien loi metrology
- rat phu hop voi STM32 + FPGA + GUI commissioning

Nhuoc diem:

- khong tu cap nhat khi tai thay doi trong luc chay
- neu current scaling sai, ket qua `Rs/Ls/Flux/J/B` se sai day chuyen
- mot so stage phu thuoc manh vao canh align, theta convention, va inverter nonlinearity

### 14.11 Ket luan ngan ve auto-tune

Neu phai tom section nay trong mot doan:

> Auto-tune cua repo nay la mot chuoi nhan dang plant offline theo dung tinh than dieu khien co so. Dau tien no do tham so dien `Rs`, `Ls`, `PolePairs`, `Flux` de co mo hinh dien va torque constant. Sau do no nhan dang co hoc `J`, `B` de co mo hinh outer loop. Cuoi cung firmware doi bandwidth mong muon thanh gain PI thong qua cac cong thuc dat gain dua tren mo hinh plant. Current PI bam rat sat sach giao khoa, speed PI dung cong thuc mo hinh + tich phan bao thu hon de ben vung thuc te, con position PI duoc tinh ra du dong bo parameter table du runtime position mode hien tai dang uu tien planner thay vi PI truc tiep.

---

## 15. Cac cau tra loi ngan gon khi thay hoi

### 15.1 Vi sao phai doi `theta = 2pi - theta_raw`

Vi convention goc duong cua encoder raw dang nguoc voi convention goc duong ma Park / Inverse Park trong lib dang dung, nen phai dao chieu de `Id/Iq` co y nghia dung.

### 15.2 Vi sao dung reduced Clarke

Vi phan cung chi do 2 dong pha, dong thu 3 duoc suy ra tu dieu kien tong 3 dong bang 0.

### 15.3 Vi sao current loop dung PI ma khong dung PID

Vi current loop trong he `dq` gan voi he bac 1 `RL`, PI thuong da du, con D-term de nhay theo nhieu.

### 15.4 Vi sao position gain tang ma khong thay doi dap ung

Vi position mode active hien tai khong dung `gPositionPi.m_calc()`, ma dung planner sinh `speed_reference_rpm`, roi dua speed reference do vao `gSpeedPi`.

### 15.5 Planner dang theo y tuong nao

Planner tinh khoang con lai, uoc luong quang phanh can thiet theo `v^2/(2a)`, tu do quyet dinh chu ky nay nen cho chay bao nhieu rpm.

---

## 16. Ket luan ngan

Neu phai tom toan bo phan trao doi nay trong vai dong:

- Hardware duoc thiet ke theo triet ly `FPGA dong bo - STM32 dieu khien - TIM8 phat PWM`.
- Firmware duoc to chuc thanh `supervisory loop` va `real-time control loop`.
- FOC cua project dung quy uoc goc rat ro, nen phan `theta`, current mapping va alignment la song con.
- Position mode active hien tai van la cascade `position planner -> speed PI -> current PI`, khong phai position PI truc tiep.
- Thu vien PI va vector transform cua project la du goc de defend phan dieu khien neu hieu dung convention va duong du lieu.
