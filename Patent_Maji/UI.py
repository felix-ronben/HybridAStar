import tkinter as tk
from tkinter import filedialog

window = tk.Tk()
window.title(u'公路平面线形自动化设计程序')
window.geometry('350x600+700+300')
window.wm_attributes('-topmost', 1)
window.resizable(0, 0)

var = tk.StringVar()
var2 = tk.StringVar()

select_path = tk.StringVar()

var.set(u'尚未建立连接...')
var2.set(u'')
min_r = tk.StringVar()
min_r.set('50')
mesh_size = tk.StringVar()
mesh_size.set('10')
min_c = tk.StringVar()
min_c.set('60')
len_s = tk.StringVar()
len_s.set('20')
angle_s = tk.StringVar()
angle_s.set('0.5')
dr_s = tk.StringVar()
dr_s.set('20')
re_path = tk.StringVar()
re_path.set('0.1')
para = tk.StringVar()
para.set('1')
turn_p = tk.StringVar()
turn_p.set('5')
sx = tk.StringVar()
sx.set('x/m')
sy = tk.StringVar()
sy.set('y/m')
syaw = tk.StringVar()
syaw.set('角度')
gx = tk.StringVar()
gy = tk.StringVar()
gyaw = tk.StringVar()


def select_file():
    selected_file_path = filedialog.askopenfilename(initialdir='./')
    select_path.set(selected_file_path)
    t.delete('1.0', 'end')
    t.insert('end', '优化边界文件路径为：'+selected_file_path)


def start_process(min_r=int(min_r.get())):
    print(min_r)


logo = tk.PhotoImage(file="2.gif")

l1 = tk.Label(window, fg='red', image=logo).place(x=20, y=10, anchor='nw')

b = tk.Button(window, bg='#DCDCDC', text=u'选择文件', font=('kaiti', 15),
              width=10, height=1, command=select_file).place(x=110, y=155, anchor='nw')

t = tk.Text(window, height=3, font=('kaiti', 15), width=30)
t.place(x=23, y=195, anchor='nw')

l2 = tk.Label(window, text=u'最小曲线半径/m：', font=('kaiti', 15), foreground='red',
              width=30, height=1).place(x=0, y=265, anchor='nw')
e1 = tk.Entry(window, textvariable=min_r, width=6, font=('kaiti', 15))
e1.place(x=230, y=265, anchor='nw')

l3 = tk.Label(window, text=u'搜索网格大小/m：', font=('kaiti', 15), foreground='red',
              width=30, height=1).place(x=0, y=295, anchor='nw')
e2 = tk.Entry(window, textvariable=mesh_size, width=6, font=('kaiti', 15))
e2.place(x=230, y=295, anchor='nw')

l4 = tk.Label(window, text=u'最短曲线长度/m：', font=('kaiti', 15), foreground='red',
              width=30, height=1).place(x=0, y=325, anchor='nw')
e4 = tk.Entry(window, textvariable=min_c, width=6, font=('kaiti', 15))
e4.place(x=230, y=325, anchor='nw')

l5 = tk.Label(window, text=u'缓和曲线长度/m：', font=('kaiti', 15), foreground='red',
              width=30, height=1).place(x=0, y=355, anchor='nw')
e5 = tk.Entry(window, textvariable=len_s, width=6, font=('kaiti', 15))
e5.place(x=230, y=355, anchor='nw')

l6 = tk.Label(window, text=u' 角度分辨率/度：', font=('kaiti', 15),
              width=30, height=1).place(x=0, y=385, anchor='nw')
e6 = tk.Entry(window, textvariable=angle_s, width=6, font=('kaiti', 15))
e6.place(x=230, y=385, anchor='nw')

l7 = tk.Label(window, text=u' 每步探索方向 ：', font=('kaiti', 15),
              width=30, height=1).place(x=0, y=415, anchor='nw')
e7 = tk.Entry(window, textvariable=dr_s, width=6, font=('kaiti', 15))
e7.place(x=230, y=415, anchor='nw')

l8 = tk.Label(window, text=u'线路输出精度/m：', font=('kaiti', 15),
              width=30, height=1).place(x=0, y=445, anchor='nw')
e8 = tk.Entry(window, textvariable=re_path, width=6, font=('kaiti', 15))
e8.place(x=230, y=445, anchor='nw')

l9 = tk.Label(window, text=u'启发项权重系数：', font=('kaiti', 15),
              width=30, height=1).place(x=0, y=475, anchor='nw')
e9 = tk.Entry(window, textvariable=para, width=6, font=('kaiti', 15))
e9.place(x=230, y=475, anchor='nw')

l10 = tk.Label(window, text=u' 转向惩罚系数 ：', font=('kaiti', 15),
              width=30, height=1).place(x=0, y=505, anchor='nw')
e10 = tk.Entry(window, textvariable=turn_p, width=6, font=('kaiti', 15))
e10.place(x=230, y=505, anchor='nw')

l11 = tk.Label(window, text=u'起点：', font=('kaiti', 15),
              width=30, height=1).place(x=-60, y=535, anchor='nw')
e11 = tk.Entry(window, textvariable=sx, width=6, font=('kaiti', 15))
e11.place(x=120, y=535, anchor='nw')
e12 = tk.Entry(window, textvariable=sy, width=6, font=('kaiti', 15))
e12.place(x=190, y=535, anchor='nw')
e13 = tk.Entry(window, textvariable=syaw, width=6, font=('kaiti', 15))
e13.place(x=260, y=535, anchor='nw')

l12 = tk.Label(window, text=u'终点：', font=('kaiti', 15),
              width=30, height=1).place(x=-60, y=535, anchor='nw')
e14 = tk.Entry(window, textvariable=x, width=6, font=('kaiti', 15))
e14.place(x=120, y=535, anchor='nw')
e15 = tk.Entry(window, textvariable=gy, width=6, font=('kaiti', 15))
e15.place(x=190, y=535, anchor='nw')
e16 = tk.Entry(window, textvariable=gyaw, width=6, font=('kaiti', 15))
e16.place(x=260, y=535, anchor='nw')

# b1 = tk.Button(window, bg='#DCDCDC', text=u'生成线路', font=('kaiti', 15),
#                width=10, height=1, command=start_process).place(x=110, y=545, anchor='nw')

window.mainloop()
