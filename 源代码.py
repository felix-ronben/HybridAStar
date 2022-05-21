import tkinter as tk
import win32com.client
import pythoncom
import time

window = tk.Tk()
window.title(u'房屋拆迁面积计算')
window.geometry('350x400+700+300')
window.wm_attributes('-topmost', 1)
window.resizable(0, 0)

var = tk.StringVar()
var2 = tk.StringVar()

var.set(u'尚未建立连接...')
var2.set(u'')


def vtpnt(x, y, z=0):  # Python和CAD数据类型不一样，需要转换
    """坐标点转化为浮点数"""
    return win32com.client.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, (x, y, z))


def vtobj(obj):  # Python和CAD数据类型不一样，需要转换
    """转化为对象数组"""
    return win32com.client.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_DISPATCH, obj)


def vtFloat(list):  # Python和CAD数据类型不一样，需要转换
    """列表转化为浮点数"""
    return win32com.client.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, list)


def vtInt(list):  # Python和CAD数据类型不一样，需要转换
    """列表转化为整数"""
    return win32com.client.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_I2, list)


def vtVariant(list):  # Python和CAD数据类型不一样，需要转换
    """列表转化为变体"""
    return win32com.client.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_VARIANT, list)


def retryCMD(cmd: str):
    while True:
        try:
            return eval(cmd)
        except:
            time.sleep(0.001)
            print('Error at ' + cmd)
            pass

acad = None


def connect():
    global on_hit
    global acad
    try:
        acad = win32com.client.Dispatch("AutoCAD.Application.24")
        doc = acad.ActiveDocument
    except:
        pass
    if acad != None:

        var.set(u'建立连接成功！')
        var2.set(u'当前文件名为：' + doc.Name)
    else:
        var.set(u'尚未建立连接...')
        var2.set(u'')


obj_new = None
on_hit2 = False
offset2 = None
offset1 = None
slt = None
rc = None


def sel_cal():
    global on_hit2
    global obj_new
    global rc
    global offset2
    global offset1
    global slt
    global new_dis
    global new_dir
    global obj
    global temp_circle
    global cors
    global i
    global temp_slt
    global slts
    if on_hit2 == False:
        on_hit2 = True
        doc = acad.ActiveDocument
        mp = doc.ModelSpace
        try:
            doc.SelectionSets.Item("SS1").Delete()
        except:
            pass

        slt = doc.SelectionSets.Add("SS1")  # 建立一个选择集叫 SS1，默认为空集
        slt.Clear()
        # win32api.MessageBox(0, '点击确定后在cad中选择线路', '提示', win32con.MB_OK)
        slt.SelectOnScreen()  # 从CAD上直接点击选择我们的路线（多段线），它会自动被加入到选择集里

        obj = retryCMD('slt[0]')   # 路线（多段线）这个对象

        # number_of_interupt = 300  # 由于路线里存在圆弧，把路线在竖直方向上分成等距的多段折线，这里数字代表分成多少段，越大对于弧线的近似越好
        line_width = float(new_dis.get())  # 线路左右需要考虑的影响宽度
        temp_cor = retryCMD('list(obj.Coordinates)')
        x_start, y_start, x_end, y_end = temp_cor[0], temp_cor[1], temp_cor[-2], temp_cor[-1]
        x_circle, y_circle = x_start, y_start
        n_iter = 0
        len_of_inter = 30
        new_center_coor = [x_start, y_start, 0]
        while True:
            temp_circle = mp.AddCircle(vtpnt(x_circle, y_circle, 0), str(len_of_inter))
            temp_inter = retryCMD('list(obj.IntersectWith(temp_circle, 0))')
            if len(temp_inter) == 6:
                old_x_circle, old_y_circle = new_center_coor[-6], new_center_coor[-5]
                if abs(temp_inter[0]-old_x_circle) + abs(temp_inter[1]-old_y_circle) < len_of_inter/10:
                    new_center_coor.append(temp_inter[3])
                    new_center_coor.append(temp_inter[4])
                    new_center_coor.append(0)
                    x_circle, y_circle = temp_inter[3], temp_inter[4]
                elif abs(temp_inter[3]-old_x_circle) + abs(temp_inter[4]-old_y_circle) < len_of_inter/10:
                    new_center_coor.append(temp_inter[0])
                    new_center_coor.append(temp_inter[1])
                    new_center_coor.append(0)
                    x_circle, y_circle = temp_inter[0], temp_inter[1]
                else:
                    print('检查交点错误！')
                    raise
            elif len(temp_inter) == 3 and n_iter == 0:
                new_center_coor.append(temp_inter[0])
                new_center_coor.append(temp_inter[1])
                new_center_coor.append(0)
                x_circle, y_circle = temp_inter[0], temp_inter[1]
            elif len(temp_inter) == 3 and n_iter > 0:
                retryCMD('temp_circle.Delete()')
                break
            else:
                print('交点个数错误！')
                raise
            n_iter += 1
            retryCMD('temp_circle.Delete()')

        new_center_coor.append(x_end)
        new_center_coor.append(y_end)
        new_center_coor.append(0)

        obj_new = mp.AddPolyline(vtFloat(new_center_coor))  # 折线化处理后的路线对象

        temp = obj_new.Offset(str(line_width))  # 路线左右偏移
        offset1 = temp[0]  # 偏移线1

        temp = obj_new.Offset(str(-line_width))  # 路线左右偏移
        offset2 = temp[0]  # 偏移线2

        # nvetex = number_of_interupt+1 #路线节点个数
        nvetex1 = int(len(list(offset1.Coordinates)) / 3)  # 路线节点个数
        nvetex2 = int(len(list(offset2.Coordinates)) / 3)  # 路线节点个数
        lineoffset1 = list(offset1.Coordinates)  # 路线左偏移后的所有节点坐标
        lineoffset2 = list(offset2.Coordinates)  # 路线右偏移后的所有节点坐标

        cors = []  # 将左偏移线和右偏移线连成一个闭合多段线，cors是这个多段线的所有节点坐标
        for i in range(nvetex1):
            cors.insert(0, 0)
            cors.insert(0, lineoffset1[3 * (nvetex1 - 1 - i) + 1])
            cors.insert(0, lineoffset1[3 * (nvetex1 - 1 - i)])

        for i in range(nvetex2):
            cors.append(lineoffset2[3 * (nvetex2 - 1 - i)])
            cors.append(lineoffset2[3 * (nvetex2 - 1 - i) + 1])
            cors.append(0)

        cors.append(cors[0])
        cors.append(cors[1])
        cors.append(cors[2])

        acad.Update()
        slt.Clear()  # 清空选择集
        retryCMD('slt.SelectByPolygon(2, vtFloat(cors))')
        retryCMD('slt.SelectByPolygon(6, vtFloat(cors))')

        n_blocks = retryCMD('slt.Count')
        sum_area = 0  # 统计总面积
        slts = []
        for i in range(n_blocks):
            temp_slt = retryCMD("slt[i]")
            if temp_slt not in [obj, obj_new, offset1, offset2]:
                retryCMD("temp_slt.Highlight(True)")  # 选择集里的对象高亮
                sum_area += retryCMD("float(slt[i].Area)")  # 面积累加
                slts.append(temp_slt)

        rc = mp.AddPolyline(vtFloat(cors))
        output = "房屋总面积为 %.1f 平米\n" % (sum_area)

        t.insert('end', output)
    else:
        on_hit2 = False
        t.delete('1.0', 'end')
        retryCMD("obj_new.Delete()")
        retryCMD("offset1.Delete()")
        retryCMD("offset2.Delete()")
        retryCMD("rc.Delete()")
        
        n_blocks = len(slts)
        for i in range(n_blocks):
            temp_slt = slts[i]
            if temp_slt not in [obj, obj_new, offset1, offset2]:
                retryCMD("temp_slt.Highlight(False)")  # 选择集里的对象高亮
        acad.Update()


l1 = tk.Label(window, text=u'目前支持CAD 2021版', bg='#98FB98', font=(
    'kaiti', 15), width=30, height=1, fg='red').place(x=20, y=10, anchor='nw')
l2 = tk.Label(window, text=u'1.先在CAD中打开文件 ', bg='#98FB98', font=(
    'kaiti', 15), width=30, height=1, fg='red').place(x=20, y=35, anchor='nw')
l3 = tk.Label(window, text=u'2.随后再点击测试连接', bg='#98FB98', font=(
    'kaiti', 15), width=30, height=1, fg='red').place(x=20, y=60, anchor='nw')
l4 = tk.Label(window, text=u'3.点击选择线路并计算', bg='#98FB98', font=(
    'kaiti', 15), width=30, height=1, fg='red').place(x=20, y=85, anchor='nw')
l5 = tk.Label(window, textvariable=var, font=('kaiti', 15),
              width=30, height=1).place(x=20, y=115, anchor='nw')
b = tk.Button(window, bg='#DCDCDC', text=u'测试连接', font=('kaiti', 15),
              width=10, height=1, command=connect).place(x=110, y=145, anchor='nw')
l6 = tk.Label(window, textvariable=var2, font=('kaiti', 15),
              width=30, height=1).place(x=20, y=185, anchor='nw')
b1 = tk.Button(window, bg='#DCDCDC', text=u'选择线路并计算', font=(
    'kaiti', 15), width=16, height=1, command=sel_cal).place(x=80, y=220, anchor='nw')

t = tk.Text(window, height=3, font=('kaiti', 15), width=30)
t.place(x=23, y=265, anchor='nw')


l7 = tk.Label(window, text=u'偏移值(m)：', font=('kaiti', 15),
              width=30, height=1).place(x=60, y=365, anchor='nw')
new_dis = tk.StringVar()
new_dis.set('36.6')
e1 = tk.Entry(window, textvariable=new_dis, width=6, font=('kaiti', 15))
e1.place(x=260, y=365, anchor='nw')
window.mainloop()
