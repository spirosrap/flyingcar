import hashlib

def hash256(string):
   hash_object = hashlib.sha256(string)
   hex_dig = hash_object.hexdigest()
   return(hex_dig)

print(hash256("Hello World"))
