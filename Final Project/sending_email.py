import serial
import smtplib
from email.mime.text import MIMEText

# ====== Configuration ======
SMTP_SERVER = "smtp.gmail.com"
SMTP_PORT = 465  # SSL port
SENDER_EMAIL = "group8ars@gmail.com"
SENDER_PASSWORD = "snmf spld wuhu bass"  # Use Gmail App Password
RECEIVER_EMAIL = "aniket.pal@mib.uni-stuttgart.de"

# Serial Port Setup (adjust COM port as necessary)
SERIAL_PORT = "COM5"  # Example: Windows = 'COM3', Linux = '/dev/ttyUSB0'
BAUD_RATE = 9600

# ====== Prepare Serial Connection ======
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
print("Listening for EMAIL_TRIGGER...")

# ====== Wait for Arduino Signal and Send Email ======
while True:
    line = ser.readline().decode('utf-8').strip()
    if line == "EMAIL_TRIGGER":
        print("Trigger received! Sending email...")

        # Compose email
        subject = "LockerNinja Breach Alert!"
        body = "A breach has been detected by LockerNinja. Please check immediately."

        msg = MIMEText(body)
        msg["Subject"] = subject
        msg["From"] = SENDER_EMAIL
        msg["To"] = RECEIVER_EMAIL

        # Send email securely via SMTP SSL
        with smtplib.SMTP_SSL(SMTP_SERVER, SMTP_PORT) as server:
            server.login(SENDER_EMAIL, SENDER_PASSWORD)
            server.sendmail(SENDER_EMAIL, RECEIVER_EMAIL, msg.as_string())

        print("Email sent successfully to", RECEIVER_EMAIL)
        break  # Exit after sending email once
