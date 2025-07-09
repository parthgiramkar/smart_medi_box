# 💊 Smart Medi Box

An intelligent pill dispensing system designed to ensure timely medication for patients, especially the elderly. It uses RFID authentication, IR sensors, stepper motors, and real-time alerts to automate the medicine-taking process.


---

## 🔩 Hardware Components Used

| Component              | Description                                |
|------------------------|--------------------------------------------|
| Arduino Uno            | Microcontroller for controlling everything |
| Stepper Motor (28BYJ-48) | For rotating compartments                 |
| ULN2003 Driver         | Stepper motor controller                    |
| RFID Module (RC522)    | Patient authentication                     |
| IR Sensor              | Detects whether medicine was picked up     |
| LCD I2C Display (16x2) | User interaction & messages                 |
| GSM Module             | Sends SMS alerts if medicine not picked    |
| Buzzer                 | Audio alerts                               |
| Push Button            | Emergency alert                            |
| Power Supply           | 5V regulated power source                   |

---


## 📁 Project Structure
```
smart_medi_box/
├── Arduino_Code/
│   └── smart_medi_box.ino
├── Images/
│   ├── block_diagram.png
│   ├── stepper_motor.jpg
│   ├── workflow_diag.png
├── .gitignore
└── README.md


```
---

## 🚀 How It Works

1. Caregiver sets up medication times via serial input.
2. At scheduled time:
   - LCD shows a message
   - RFID scan is required to dispense medicine
   - Stepper motor rotates to deliver medicine
3. IR sensor waits for medicine to be picked up.
4. If not picked within 1 minute, buzzer alerts and SMS is sent.

---

## 📞 Emergency Alert Feature

- If the emergency button is pressed:
  - Buzzer sounds
  - Emergency message is displayed
  - (Optional: SMS can be sent to caregiver)

---

## 🛠 How to Run

1. Open `Arduino_Code/smart_medi_box.ino` in Arduino IDE
2. Install required libraries:
   - `LiquidCrystal_I2C`
   - `Stepper`
   - `MFRC522` (for RFID)
3. Upload to Arduino Uno
5. Monitor serial terminal for input prompts

---

## 💡 Made as part of academic and real-world health project integration



