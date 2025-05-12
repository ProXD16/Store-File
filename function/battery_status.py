import requests
import json
import tkinter as tk

# IP và host của robot
ip = '192.168.0.172'
host = 'http://' + ip + '/api/v2.0.0/'

# Headers cho yêu cầu API
headers = {
    'Content-Type': 'application/json',
    'Authorization': 'Basic YWRtaW46OGM2OTc2ZTViNTQxMDQxNWJkZTkwOGJkNGRlZTE1ZGZiMTY3YTljODczZmM0YmI4YTgxZjZmMmFiNDQ4YTkxOA=='
}

# Khởi tạo cửa sổ Tkinter
root = tk.Tk()
root.title("Battery Status")

# Biến để lưu giá trị trạng thái pin
battery_percentage_value = tk.StringVar()

# Textbox hiển thị trạng thái pin
battery_percentage_display = tk.Entry(root, textvariable=battery_percentage_value, font=('Arial', 14), width=20)
battery_percentage_display.pack(pady=20)

# Hàm lấy trạng thái pin từ API và cập nhật vào GUI
def pin():
    global host, headers, battery_percentage_value, battery_percentage_display
    try:
        # Gửi yêu cầu tới API
        b = requests.get(host + '/status', headers=headers)

        # Kiểm tra xem yêu cầu có thành công không
        if b.status_code == 200:
            a = json.loads(b.content)
            p = a['battery_percentage']

            # Cập nhật màu sắc hiển thị dựa trên trạng thái pin
            battery_percentage_display.configure(background="lightgray")
            if p <= 15:
                battery_percentage_display.configure(foreground="red")
            else:
                battery_percentage_display.configure(foreground="blue")

            # Hiển thị giá trị pin
            value = round(p, 2)
            c = f'{value}%'
            battery_percentage_value.set(c)
        else:
            print("Failed to retrieve battery status.")
    
    except Exception as e:
        print(f"Error occurred: {e}")
    
    # Lặp lại hàm pin sau mỗi 5000ms (5 giây)
    root.after(5000, pin)

# Gọi hàm pin để bắt đầu lấy dữ liệu pin
pin()

# Bắt đầu vòng lặp Tkinter
root.mainloop()
