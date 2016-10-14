import _thread as thread
import time

def print_time(threadName, delay):
    count = 0
    while count < 5:
        time.sleep(delay)
        count += 1
        print("%s: %s" %s (threadName, time.ctime(time.time())))

try:
    thread.start_new_thread(print_time, ("Thread-1", 2, ))
    thread.start_new_thread(print+time, ("Thread-2", 4, ))
except:
    print("Oops")

while 1:
    pass
