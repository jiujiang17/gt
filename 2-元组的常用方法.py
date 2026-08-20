# words = "abcdefghi"
# a = words[2:7:2]
# print(a)
# b = words[-1:-10:-1]
# print(b)
# c = words[0:9:2]
# print(c)
# d = words[-1:-10:-1]
# print(d)
#
# list1 = [1,11,43,67,5,6]
# list1[3]='小明'
# print(list1)
# print(list1.index(43))
# list1.pop(4)
# print(list1)
# name = input("输入姓名：")
# print("{}同学，学好python,前途无量".format(name))
# from datetime import datetime
# now = datetime.now()
# print(now)
# n = input("n等于：")
# sum = 0
# for i in range(int(n)):
#     sum += i+1
# print("1到n为：{}".format(sum))

# for i in range(1,10):
#     for j in range(1,i+1):
#         print("{}*{}={:2}".format(j,i,j*i),end=" ")
#     print(" ")


# sum_total = 0
# for i in range(1,11):
#     sum = 1
#     for j in range(1,i+1):
#         sum *= j
#     sum_total += sum

# sum , temp = 0 , 1
# for i in range(1,11):
#     temp *= i
#     sum += temp
# print(sum)

# m , n = map(int,input("第_天还剩_个桃子").split())
# for i in range(int(m-1)):
#     n = (n+1)*2
# print(n)

# from turtle import *
# fillcolor('red')
# begin_fill()
# while 1 :
#     forward(200)
#     right(144)
#     if abs(pos())<1:
#         break
#
# def CurrencyExchanger(ValueStr):
#     if ValueStr[-1] in "刀":
#         M = (eval(ValueStr[:-1]))*6
#         print("转换后是：{:.2f}元".format(M))
#     elif ValueStr[-1] in "元":
#         C = (eval(ValueStr[:-1]))/6
#         print("转换后是：{:.2f}刀".format(C))
#     else :
#         print("输入格式错误")
#
# TempStr = input("请输入你想换算的金额：")
# CurrencyExchanger(TempStr)

# import turtle
# turtle.seth(90)
# i = 1
# while 1:
#     turtle.forward(i)
#     i += 5
#     turtle.left(90)

# a = 1
# for i in range(365):
#     a *= 1.01
# def Dayup(b):
#     a = 1
#     for i in range(365):
#         if i%7 in [0,6]:
#             a *= 0.99
#         else:
#             a *= 1+b
#     return a
# c = 0.001
# while(Dayup(c)<37.78):
#     c += 0.0001
# print("{:.3}".format(c))
# def Day_up(n):
#     a = 13
#     for i in range(365):
#         if i%7 not in [5,6,0]:
#             a += n
#     return a
# b = 0.002
# print("{:.3}".format(Day_up(b)))

# str = "星期一星期二星期三星期四星期五星期六星期天"
# pos = (eval(input("今天是星期几："))-1)*3
# print("今天是{}".format(str[pos:pos+3]))

# plaincode = input("请输入想加密的内容：")
# # for i in plaincode:
# #     if ord("a")<=ord(i) and ord(i)<=ord('z'):
# #         print(chr(ord("a")+((ord(i)-ord("a")+3)%26)),end="")
# #     else:
# #         print(i,end="")
#
# for i in plaincode:
#     if ord("a")<=ord(i) and ord(i)<=ord('z'):
#         print(chr(ord("a")+((ord(i)-ord("a")-3)%26)),end="")
#     else:
#         print(i,end="")

# a = 70
# for i in range(1,11):
#     print("第{}年在地球上的体重为：{}千克，在月球上的体重为：{}千克".format(i,a+i*0.5,(a+i*0.5)*0.165))
# a = 1
# b = 0
# flag = 0
# for i in range(1,366):
#     if i%11 == 0:
#         b = i
#     else:
#         if i-b > 3:
#             a *= 1.01
# print(a)

# try:
#     num = eval(input("请输入一个整数:"))
#     print(num**2)
# except :
#     print("输入错误")

# a , b = 0 , 0
# while 1 :
#     a =eval(input("请输入一个整数:"))
#     b += 1
#     if a == 3 :
#         break
#     elif a > 3 :
#         print("大了")
#     elif a < 3 :
#         print("小了")
# print("预测了{}次，你猜中了！".format(b))

# from random import *
# r = [0,0,1]
# w = 0
# for i in range(100000):
#     shuffle(r)
#     if r[1]==0:
#         if r[2]==1:
#             w += 1
#     elif r[2]==0:
#         if r[1]==1:
#             w += 1
# print("交换后胜率为{}".format(w/100000))

# from datetime import datetime
# now = datetime(2026,6,9,21,5,15,456123)
# print(now)

a = "cat","dog","tigger"
print(a)
