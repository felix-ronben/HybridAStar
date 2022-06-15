import win32com.client
import pythoncom
import time
import xlrd
import xlsxwriter as xw
import matplotlib.pyplot as plt

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

try:
    acad = win32com.client.Dispatch("AutoCAD.Application.24")
    doc = acad.ActiveDocument
except:
    pass

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
line_width = 50  # 线路左右需要考虑的影响宽度
temp_cor = retryCMD('list(obj.Coordinates)')
x_start, y_start, x_end, y_end = temp_cor[0], temp_cor[1], temp_cor[-2], temp_cor[-1]
x_circle, y_circle = x_start, y_start
n_iter = 0
len_of_inter = 5
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

workbook = xw.Workbook('line_points.xlsx')  # 创建工作簿
worksheet1 = workbook.add_worksheet("sheet1")  # 创建子表
worksheet1.activate()  # 激活表
i = 1
for j in range(nvetex1):
    insertData = [round(lineoffset1[3 * j],2),round(lineoffset1[3 * j+1],2)]
    row = 'A' + str(i)
    worksheet1.write_row(row, insertData)
    i += 1
    plt.plot(round(lineoffset1[3 * j],2),round(lineoffset1[3 * j+1],2),'or')
for j in range(nvetex2):
    insertData = [round(lineoffset2[3 * j],2),round(lineoffset2[3 * j+1],2)]
    row = 'A' + str(i)
    worksheet1.write_row(row, insertData)
    i += 1
    plt.plot(round(lineoffset2[3 * j],2),round(lineoffset2[3 * j+1],2),'or')

x_start, y_start, x_end, y_end = lineoffset1[0], lineoffset1[1], lineoffset2[0], lineoffset2[1]
x_circle, y_circle = x_start, y_start
n_iter = 0
new_center_coor = [x_start, y_start, 0]
obj_left = mp.AddPolyline(vtFloat([lineoffset1[0], lineoffset1[1], 0, lineoffset2[0], lineoffset2[1],0]))
while True:
    temp_circle = mp.AddCircle(vtpnt(x_circle, y_circle, 0), str(len_of_inter))
    temp_inter = retryCMD('list(obj_left.IntersectWith(temp_circle, 0))')
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

for j in range(int(len(new_center_coor) / 3)):
    insertData = [round(new_center_coor[3 * j],2),round(new_center_coor[3 * j+1],2)]
    row = 'A' + str(i)
    worksheet1.write_row(row, insertData)
    i += 1
    plt.plot(round(new_center_coor[3 * j],2),round(new_center_coor[3 * j+1],2),'or')

x_start, y_start, x_end, y_end = lineoffset1[-3], lineoffset1[-2], lineoffset2[-3], lineoffset2[-2]
x_circle, y_circle = x_start, y_start
n_iter = 0
new_center_coor = [x_start, y_start, 0]
obj_right = mp.AddPolyline(vtFloat([lineoffset1[-3], lineoffset1[-2], 0, lineoffset2[-3], lineoffset2[-2],0]))
while True:
    temp_circle = mp.AddCircle(vtpnt(x_circle, y_circle, 0), str(len_of_inter))
    temp_inter = retryCMD('list(obj_right.IntersectWith(temp_circle, 0))')
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

for j in range(int(len(new_center_coor) / 3)):
    insertData = [round(new_center_coor[3 * j],2),round(new_center_coor[3 * j+1],2)]
    row = 'A' + str(i)
    worksheet1.write_row(row, insertData)
    i += 1
    plt.plot(round(new_center_coor[3 * j],2),round(new_center_coor[3 * j+1],2),'or')

workbook.close()  # 关闭表
plt.show()
retryCMD("obj_new.Delete()")
retryCMD("offset1.Delete()")
retryCMD("offset2.Delete()")
retryCMD("obj_left.Delete()")
retryCMD("obj_right.Delete()")