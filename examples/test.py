import os

user = os.getenv("USER")
sudo_user = os.getenv("SUDO_USER")
print(f"user {user} - sudo_user {sudo_user}")

user = os.getenv("SUDO_USER", os.getenv("USER"))
print(f"user {user}")

os.system(f"systemctl daemon-reload")