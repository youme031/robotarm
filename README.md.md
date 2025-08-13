# BLDC Driver (BDD-480) - STM32F7 PWM Control (Slow/Stable)

## 핵심 기능
- BDD-480 BLDC 드라이버 VSP에 PWM 인가해 속도 제어
- DIR 핀으로 회전 방향 제어
- 듀티 한계 가드: MIN 0.1%, MAX 20%

## 핀 연결
- VSP  ←→ PE9 (TIM1_CH1, PWM)
- DIR  ←→ PB0 (GPIO Output)
- EN   ←→ GND (항상 Enable)
- GND  ←→ GND (공통 접지)
- U/V/W ↔ 모터 3상 (기존과 동일)

## 클럭/타이머 설정 (CubeIDE .ioc 기준)
- System Clock: 216 MHz (HSE 25MHz → PLL: M=25, N=432, P=/2, Q=/9, R=/2)
- Power OverDrive: Enabled
- AHB: /1 → 216 MHz
- APB1: /4, APB2: /2
- **TIM1 Timer Clock**: APB2가 /2이므로 Timer x2 → **216 MHz**

### TIM1 (PWM)
- Channel: CH1 (PE9)
- Prescaler (PSC): **215**
- Period (ARR): **999**
- PWM Mode: PWM1
- Auto-reload preload: Enable
- **PWM Frequency**: 216MHz/(PSC+1)/(ARR+1) = **1 kHz**
- 초기 듀티: `TIM1_SetDuty(0.20f)` → 20%

## 빌드/동작
1. 보드와 BDD-480을 위 핀대로 연결 (EN은 GND 고정).
2. 펌웨어 플래시 후 리셋.
3. 모터가 DIR=High 방향으로 **1 kHz PWM, 듀티 20%**로 구동.

## 현장 조정 포인트
- 느리게: `TIM1_SetDuty(0.005f);`  // 0.5%
- 빠르게: `TIM1_SetDuty(0.020f);`  // 2%
- 범위를 바꾸려면 `DUTY_MIN`, `DUTY_MAX` 수정.
- **주파수를 바꾸고 싶다면** TIM1 PSC/ARR 변경:
  - 예) 5 kHz: PSC=215 그대로, ARR=199 → 1MHz/200=5kHz
  - 예) 20 kHz: PSC=215 그대로, ARR=49  → 1MHz/50 = 20kHz

## 참고/주의
- PWM 극성이 드라이버 입력과 반대로 먹는 경우 `#define PWM_INVERT 1`로 변경.
- RNG는 현재 미사용(초기화만 있음). 정리 필요 시 `MX_RNG_Init()`과 관련 핸들 제거 가능.
