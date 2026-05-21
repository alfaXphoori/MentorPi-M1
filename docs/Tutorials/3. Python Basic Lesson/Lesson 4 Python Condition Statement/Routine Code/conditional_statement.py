height = float(input("height(m)："))
weight = float(input("weight(kg)："))

bmi = weight / (height ** 2)   #calculate BMI
print("BMI："+str(bmi))

if bmi<18.5:
    print("体重过轻")
elif bmi>=18.5 and bmi<24.9:
    print("正常范围，注意保持")
elif bmi>=24.9 and bmi<29.9:
    print("体重过重")
else:
    print("肥胖")

