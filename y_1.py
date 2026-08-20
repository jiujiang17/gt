#append 加到列表的末尾
#insert 加到指定位置
#extend 将其他列表的元素依次加到该列表的末尾
a=[3,2,2,8]
a.append(4)
print(a)
a.insert(2,5)
print(a)
b=[288,255]
a.extend(b)
print(a)

'''
删除的方法
    语法1：del 列表[索引]
    语法2：pop(数字) a.pop()删去对应的元素
    语法3：remove() 删除第一个的对应元素
    语法4：clear() 全删
'''
c=['甲','乙','丙','丁']
del c[1]
print(c)
c.pop(1)
print(c)
c.remove('丁')
print(c)
c.clear()
print(c)

'''
查询的方法
    语法1：index() 返回值
    语法2：count() 列表中某个元素的个数
    语法3：len() 数列表的有多少个元素
'''
d=['甲','乙','丙','丁','甲']
e=d.index('丁')
print(e)#e=3
e=d.count('甲')
print(e)#e=2
print(len(d))#5



