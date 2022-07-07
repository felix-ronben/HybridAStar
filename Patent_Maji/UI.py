import tkinter as tk
from tkinter import filedialog

window = tk.Tk()
window.title(u'公路平面线形自动化设计程序')
window.geometry('350x430+700+300')
window.wm_attributes('-topmost', 1)
window.resizable(0, 0)

var = tk.StringVar()
var2 = tk.StringVar()

select_path = tk.StringVar()

var.set(u'尚未建立连接...')
var2.set(u'')
min_r = tk.StringVar()
min_r.set('（m）')
mesh_size = tk.StringVar()
mesh_size.set('（m）')
min_c = tk.StringVar()
min_c.set('（m）')
len_s = tk.StringVar()
len_s.set('（m）')


def select_file():
    selected_file_path = filedialog.askopenfilename(initialdir='./')
    select_path.set(selected_file_path)
    t.delete('1.0', 'end')
    t.insert('end', '优化边界文件路径为：'+selected_file_path)


def start_process():
    pass


logo = tk.PhotoImage(file="2.gif")

l1 = tk.Label(window, fg='red', image=logo).place(x=20, y=10, anchor='nw')

b = tk.Button(window, bg='#DCDCDC', text=u'选择文件', font=('kaiti', 15),
              width=10, height=1, command=select_file).place(x=110, y=155, anchor='nw')

t = tk.Text(window, height=3, font=('kaiti', 15), width=30)
t.place(x=23, y=195, anchor='nw')

l2 = tk.Label(window, text=u'最小曲线半径：', font=('kaiti', 15),
              width=30, height=1).place(x=0, y=265, anchor='nw')
e1 = tk.Entry(window, textvariable=min_r, width=6, font=('kaiti', 15))
e1.place(x=230, y=265, anchor='nw')

l3 = tk.Label(window, text=u'搜索网格大小：', font=('kaiti', 15),
              width=30, height=1).place(x=0, y=295, anchor='nw')
e2 = tk.Entry(window, textvariable=mesh_size, width=6, font=('kaiti', 15))
e2.place(x=230, y=295, anchor='nw')

l4 = tk.Label(window, text=u'最短曲线长度：', font=('kaiti', 15),
              width=30, height=1).place(x=0, y=325, anchor='nw')
e4 = tk.Entry(window, textvariable=min_c, width=6, font=('kaiti', 15))
e4.place(x=230, y=325, anchor='nw')

l5 = tk.Label(window, text=u'缓和曲线长度：', font=('kaiti', 15),
              width=30, height=1).place(x=0, y=355, anchor='nw')
e5 = tk.Entry(window, textvariable=len_s, width=6, font=('kaiti', 15))
e5.place(x=230, y=355, anchor='nw')

b1 = tk.Button(window, bg='#DCDCDC', text=u'生成线路', font=('kaiti', 15),
               width=10, height=1, command=start_process).place(x=110, y=385, anchor='nw')

window.mainloop()
