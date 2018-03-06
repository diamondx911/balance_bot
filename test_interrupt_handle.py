import signal
import sys
import time

def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
while True:
    start = time.time()
    print("I'm ali")
    time.sleep(1)
    end = time.time()
    print(end-start)
    
#print('Press Ctrl+C')
#signal.pause()