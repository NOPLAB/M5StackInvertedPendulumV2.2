import socket
import json

Host = "192.168.1.2"
Port = 65000

objMsg = {}

while True:
    print("Type:")
    type = input()
    if type == "end" or type == "e":
        print("end")
        break
    print("Value:")
    value = input()
    if type == "ap":
        objMsg.update(ap=value)
    elif type == "ai":
        objMsg.update(ai=value)
    elif type == "ad":
        objMsg.update(ad=value)
    elif type == "pp":
        objMsg.update(pp=value)
    elif type == "pi":
        objMsg.update(pi=value)
    elif type == "pd":
        objMsg.update(pd=value)
    else:
        print("Missing Type")

msg = json.dumps(objMsg)

print(msg)

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

client.connect((Host, Port))

client.send(msg.encode("ascii"))

receiveMsg = client.recv(1024).decode("ascii")

print(receiveMsg)

client.close()
