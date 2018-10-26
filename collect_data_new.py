from time import sleep
import xbox
import thread
import csv
from Camera.camera import Camera
from Controller.controller import Controller

def main():
	try:
            i = 0
            history = []
            carCtrl = Controller()
            camCtrl = Camera()
            thread.start_new_thread(carCtrl.steering,())
            while True:
        
                
                #camCtrl.save_img(camCtrl.capture())
                camCtrl.make_img(i)
                history.append(carCtrl.control_order)
                i += 1
                
                # check if stop capturing
                if carCtrl.control_sequence == False:
                    print("Checked if control_sequence is still alive :: No!")
                    break
        
            with open("Data/history.csv","w") as csvfile:
                filecsv = csv.writer(csvfile, delimiter = ",")
                for command in history:
                    filecsv.writerow([command])
        
        except OSError as err:
            print(err)
            pass
            
        finally:
            #camCtrl.close_cam()
            pass




if __name__ == '__main__':
	main()

    
