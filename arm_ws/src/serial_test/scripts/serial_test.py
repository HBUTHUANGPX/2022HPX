import yaml
import os
def write(yaml_file):
  py_object = {'data': c}
  file = open(yaml_file, 'w', encoding='utf-8')
  yaml.dump(py_object, file)
  file.close()
  
a=b'\x01\x02'
c=[]
for i in a:
    c.append(int(i))
print(c)

# 写
current_path = os.path.abspath(".")
yaml_path = os.path.join(current_path, "generate.yaml")
write(yaml_path)
# 读
current_path = os.path.abspath(".")
yaml_path = os.path.join(current_path, "generate.yaml")
file = open(yaml_path, 'r', encoding="utf-8")
file_data = file.read() #读取file内容
file.close()
#*****************转化为字典***************
data = yaml.safe_load(file_data) #转换为字典类型
print(type(data))
print(data['data'])
