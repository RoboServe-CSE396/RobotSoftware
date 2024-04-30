#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import tf_transformations

import firebase_admin
from firebase_admin import credentials, firestore
from tkinter import Tk, Button

# Initialize Firebase Admin SDK
cred = credentials.Certificate("serviceAccountKey.json")
firebase_admin.initialize_app(cred)
db = firestore.client()

# Global variable for storing ordered entries
goals_entries = []
goals_ref = None
goals_col = None
is_initialized = False
nav = None
robotCommands_ref = None
robotCommands_col = None
robotCommands_entries = None

currentlyProcessedOrder_ref = None
currentlyProcessedOrder_col = None
currentlyProcessedOrder_entries = None


col_query2 = None

# Added functionality: Stop Automation button functionality is completed.

    
def handle_commands():
    global robotCommands_ref, robotCommands_col, goals_ref, goals_col, nav, col_query2
    
    
    for docx in robotCommands_ref:
        # Retrieve the first document
        current_doc_id = docx.id
        data = db.collection("robotCommands").document(current_doc_id).get().to_dict()
        command = data.get("command")
            
        if command == "Start Automation":
            db.collection("robotCommands").document(current_doc_id).delete()
            print("Process the command: Start Automation")
            col_query2 = goals_col.on_snapshot(on_snapshot_goals)
            
        elif command == "Stop Automation":
        	# Cancel the goal
            nav.cancelTask()   
            handle_stop_automation()
            col_query2.unsubscribe()
            # table0'a don
            go_to_initial_location()
            
            
              	
        break	
    




def create_pose_stamped(position_x, position_y, rotation_z):
    global nav
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = position_x
    goal_pose.pose.position.y = position_y
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose

    
def initialize_initial_pose(position_x, position_y, rotation_z):
    global nav
	# --- Set initial pose ---
    # !!! Comment if the initial pose is already set !!!
    initial_pose = create_pose_stamped(position_x, position_y, rotation_z)
    nav.setInitialPose(initial_pose)

    # --- Wait for Nav2 ---
    nav.waitUntilNav2Active()
    
    
def go_to_goal(position_x, position_y, rotation_z):
    global nav
    # --- Create some Nav2 goal poses ---
    goal_pose = create_pose_stamped(position_x, position_y, rotation_z)

    # --- Going to one pose ---
    nav.goToPose(goal_pose)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
            
    # --- Get the result ---
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
        handle_successfulNav()
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!') 
   
   
def handle_stop_automation():  # arrived yap ve current goal sil, + 
    global currentlyProcessedOrder_ref, currentlyProcessedOrder_col, currentlyProcessedOrder_entries, goals_ref, goals_col
    
    
    # Get only the first order from the orders list, thats why break is put immediately
    for docx in goals_ref:
        # Retrieve the first document
    
        current_doc_id = docx.id
        data = goals_col.document(current_doc_id).get().to_dict()
        
        # Query Firestore to find the document where OrderID is equal to "id1"
        query = currentlyProcessedOrder_col.where("OrderID", "==", data.get("OrderID")).get()

        # Check if any documents match the query
        if query:
            for doc in query:
                new_id = doc.id
                currentlyProcessedOrder_col.document(new_id).delete()
                print("Deleted in currentlyProcessedOrder!")
                break
            
            
        else:
            print("No document found with OrderID equal to 'OrderID' in currently processed orders")
       
       
        goals_col.document(current_doc_id).delete()
        break
    else:
        print("No documents found")

   
def handle_successfulNav():  # arrived yap ve current goal sil, + 
    global currentlyProcessedOrder_ref, currentlyProcessedOrder_col, currentlyProcessedOrder_entries, goals_ref, goals_col
    
    
    # Get only the first order from the orders list, thats why break is put immediately
    for docx in goals_ref:
        # Retrieve the first document
    
        current_doc_id = docx.id
        data = goals_col.document(current_doc_id).get().to_dict()
        
        # Query Firestore to find the document where OrderID is equal to "id1"
        query = currentlyProcessedOrder_col.where("OrderID", "==", data.get("OrderID")).get()

        # Check if any documents match the query
        if query:
            for doc in query:
                new_id = doc.id
                currentlyProcessedOrder_col.document(new_id).update({"orderStatus": "Arrived"})
                print("Updated in currentlyProcessedOrder!")
                break
            
            
        else:
            print("No document found with OrderID equal to 'OrderID' in currently processed orders")
       
       
        goals_col.document(current_doc_id).delete()
        break
    else:
        print("No documents found")

  
    
    
def printCommands_entry():   
    global robotCommands_entries
    
    for entry in robotCommands_entries:
    	print(entry)
    

def printGoals_entry():   
    global goals_entries  
    
    for entry in goals_entries:
    	print(entry)
    
    
def printProcessedOrders_entry():   
    global currentlyProcessedOrder_entries  
    
    for entry in currentlyProcessedOrder_entries:
    	print(entry)    
    

def refresh_currProcessedOrders():
    global currentlyProcessedOrder_ref, currentlyProcessedOrder_col, currentlyProcessedOrder_entries
    
    currentlyProcessedOrder_col = db.collection("currentlyProcessedOrder")
    
    currentlyProcessedOrder_ref = currentlyProcessedOrder_col.order_by("timestamp").get()
    currentlyProcessedOrder_entries = [doc.to_dict() for doc in currentlyProcessedOrder_ref]
    print("currentlyProcessedOrders are refreshed.")
    print(len(currentlyProcessedOrder_entries))



def refresh_goals():
    global goals_entries, goals_ref, goals_col
    

    goals_col = db.collection("goals")
    
    goals_ref = goals_col.order_by("timestamp").get()
    goals_entries = [doc.to_dict() for doc in goals_ref]
    print("Goals are refreshed.")
    print(len(goals_entries))
        
    

def refresh_commands():
     global robotCommands_ref, robotCommands_entries,robotCommands_col
     
     robotCommands_col = db.collection("robotCommands")
    
     
     robotCommands_ref = robotCommands_col.order_by("timestamp").get()
     robotCommands_ref = list(reversed(robotCommands_ref))
     robotCommands_entries = [doc.to_dict() for doc in robotCommands_ref]
     print("Commands are refreshed.")
     print(len(robotCommands_entries))
     

    
   
    

def initialize_position():
    # Query Firestore to find the document where OrderID is equal to "id1"
    query = db.collection("initialPosition").where("location", "==","initial").get()
        
    # Check if any documents match the query
    if query:
        for doc in query:
            new_id = doc.id
            data = db.collection("initialPosition").document(new_id).get().to_dict()
            initialize_initial_pose(float(data.get("position_x")),float(data.get("position_y")),float(data.get("rotation_z")))
            print("!!!!!")
            break
        
        
    else:
        print("error")           
 
   
    
def check_for_goal():
    global goals_ref, is_initialized, goals_entries, goals_col
	
    if not is_initialized:
        # initialize here
        initialize_position()
        is_initialized = True
		
		
    # Initialize a counter for the number of documents
    num_documents = 0

    # Iterate through the documents and increment the counter
    for doc in goals_ref:
        num_documents += 1
        
        
    if num_documents > 0:
    # Get only the first order from the orders list, thats why break is put immediately
        for docx in goals_ref:
            # Retrieve the first document
        
            current_doc_id = docx.id
            data = db.collection("goals").document(current_doc_id).get().to_dict()
            
            # go to goal
            # Query Firestore to find the document where OrderID is equal to "id1"
            query = db.collection("initialPosition").where("location", "==",data.get("fromWhichTable")).get()
            
            # Check if any documents match the query
            if query:
                for doc in query:
                    new_id = doc.id
                    datax = db.collection("initialPosition").document(new_id).get().to_dict()
                    go_to_goal(float(datax.get("position_x")),float(datax.get("position_y")),float(datax.get("rotation_z")))
                    print("!!!!!")
                    # remove goal
                    db.collection("goals").document(current_doc_id).delete()
                    break
        
        
            else:
                print("error")
           
            break
    
    
    else:
        print("No goal found")    
    
    

def delete_all_documents(collection_name):
    ref = db.collection(collection_name).get()
    for doc in ref:
        doc.reference.delete()



def on_snapshot_commands(col_snapshot, changes, read_time):
    try:
        for change in changes:
            if change.type.name == 'ADDED':
                refresh_commands()
                printCommands_entry()
                handle_commands()
             
            elif change.type.name == 'MODIFIED':
                refresh_commands()
                printCommands_entry()
            elif change.type.name == 'REMOVED':
                refresh_commands()
                printCommands_entry()

    except Exception as e:
        print("Error in on_snapshot:", e)

def on_snapshot_goals(col_snapshot, changes, read_time):
    try:
        for change in changes:
            if change.type.name == 'ADDED':
                refresh_goals()
                printGoals_entry()
                check_for_goal()
                
            elif change.type.name == 'MODIFIED':
                refresh_goals()
                printGoals_entry()

            elif change.type.name == 'REMOVED':
                refresh_goals()
                printGoals_entry()
            

    except Exception as e:
        print("Error in on_snapshot:", e)

def on_snapshot_currOrders(col_snapshot, changes, read_time):
    try:
        for change in changes:
            if change.type.name == 'ADDED':
                refresh_currProcessedOrders()
                printProcessedOrders_entry()
               
                
            elif change.type.name == 'MODIFIED':
                refresh_currProcessedOrders()
                printProcessedOrders_entry()
                # check for delivered
                handle_delivered()

            elif change.type.name == 'REMOVED':
                refresh_currProcessedOrders()
                printProcessedOrders_entry()
            

    except Exception as e:
        print("Error in on_snapshot:", e)


def handle_delivered(): #  delivered varsa currentProcessOrder sil ve alreadyDeliveredOrders 'a ekle ve table0'a don 
    global currentlyProcessedOrder_ref, currentlyProcessedOrder_col, currentlyProcessedOrder_entries
    query = currentlyProcessedOrder_col.where("orderStatus", "==", "Delivered").get()
    # Check if any documents match the query
    if query:
        for doc in query:
            new_id = doc.id
            order_data = currentlyProcessedOrder_col.document(new_id).get().to_dict()
            
            new_data = {
                "OrderID": order_data.get("OrderID"),
                "Price": order_data.get("Price"),
                "timestamp": firestore.SERVER_TIMESTAMP
            }

            db.collection("alreadyDeliveredOrders").add(new_data)
            currentlyProcessedOrder_col.document(new_id).delete()
            
            # table0'a don
            go_to_initial_location()
            
            
            
            break
    
    
    else:
        print("No document found with orderStatus equal to 'Delivered'")


def go_to_initial_location():
    query = db.collection("initialPosition").where("location", "==", "initial").get()
    if query:
        for doc in query:
            new_id = doc.id
            pos_data = db.collection("initialPosition").document(new_id).get().to_dict()
            go_to_goal(float(pos_data.get("position_x")), float(pos_data.get("position_y")), float(pos_data.get("rotation_z")))
            break
    else:
        print("No document found with location equal to 'initial'")



# Create a listener for real-time updates to the Firestore collection
def monitor_collection():
    global robotCommands_col, goals_col, currentlyProcessedOrder_col
    robotCommands_col = db.collection("robotCommands")
    goals_col = db.collection("goals")
    currentlyProcessedOrder_col = db.collection("currentlyProcessedOrder")
    # Retrieve documents from Firestore in descending order of timestamp
    
    
    # Listen for changes in the collection
    col_query1 = robotCommands_col.on_snapshot(on_snapshot_commands)
    delete_all_documents("robotCommands")
    
    # Listen for changes in the collection
    col_query2 = goals_col.on_snapshot(on_snapshot_goals)
    delete_all_documents("goals")
    col_query2.unsubscribe()
    
    # Listen for changes in the collection
    col_query3 = currentlyProcessedOrder_col.on_snapshot(on_snapshot_currOrders)
    delete_all_documents("currentlyProcessedOrder")
    
    
    
    
    
    
  

def main():
    global nav
    # --- Init ROS2 communications and Simple Commander API ---
    rclpy.init()
    nav = BasicNavigator()
    monitor_collection()
    
    #refresh_goals()
    #check_for_goal()
    
    # Start monitoring the Firestore collection for real-time updates
    
    input("yo")
	
    #rclpy.shutdown()

if __name__ == '__main__':
    main()   
    




# yukarıda go_to_goal fonksiyonunda hedefe varıldıktan sonra robotCommands collectionundaki butun datayı temizle
# eger tamamlanan goal'in fromWhichTable datası table0 degilse, sonraki goal kesinlikle table0 olmalı.
# table0'ı bu script içinde hallet, database'e restaurant appde ekleme.

# cc nin icinde: "delivered" oldugunda goals'deki goal'ı silmeli ve table0'a gitmeli, goals'de zaten bir değişim olmayacak. Ayrıca "Arrived" ve "Delivered" arasında robot beklemeli.
# cc nin icinde: Goal cancel oldugunda ilgili orderID'nin durumunu "Cancelled" yap. Goal basarili oldugunda ilgili orderID'nin durumunu "Arrived" yap.




# goals'in icindeki butun goal'ler table1, table2, table3, table4, veya table5 olmali. Yani orderdan dolayi olmali.  +
# goal id ve order id esit olmali + 
# Restaurant app'de, currentlyProcessedOrder'daki herhangi bir datanın status'u "Cancelled" veya "Delivered" olduysa, bu datayı sil +
# robotCommands collection her zaman dinlenmeli + 
# robotCommands'i timestamp'e gore tersten sırala, yeni gelenler her zaman oncelikli olarak dinlenmeli +
# okunan command, commande baslamadan once silinmeli + 
# goals ve robotCommands ayrı ayrı dinlemeyi duzelt +






# mobil app icinde: mobil app currentlyProcessedOrder bunu dinlemeli. mobil appde durum "Arrived" oldugunda, kullanıcı teslim aldın mı? yazan soruya ok diyince "Delivered" yap, 1dk bekle,currentlyProcessedOrder'da datayı sil, mobil appde ana sayfaya don. Böyle bir soru yoksa(teslim aldın mı?), 1 dk bekle, currentlyProcessedOrder'de durumu "Delivered" yap ve mobil app'de ana sayfaya geri don.
# mobil app icinde: durum "Cancelled" olduysa, 1dk sonra ana sayfaya don.



# start processing goals  ->  goals collection'ınını dinlemeye basla, currentlyProcessedOrder dinlemeye basla, Arrived yap, Delivered oldu mu onu dinle. Olunca geri don   startGoals +
# stop processing goals    -> cancel current task(1) simdiki goal'ı sil(2) goali sildiginde currentlyProcessedOrder'i de ara, goal id'de bir sey varsa sil, goals collection'ınını dinlemeyi bırak(1), baslangic yerine don(3)    stopGoals
# exit  -> goals'de bir goal varsa onu sil(1) sonra rclpy.shutdown() yaz.   exit

# stop current operation  -> cancel current task(2), simdiki goal'ı silme , goals collection'ınını dinlemeyi bırak(1), currentlyProcessedOrder'da goal id'li order'ı ara,varsa durumunu guncelle "Stopped", stopCurrentGoal
# cancel current operation -> cancel current task(2) ve simdiki goal'ı sil(3), table0'a git(4),  goals collection'ınını dinlemeyi bırak(1)    Goal cancel oldugunda ilgili orderID'nin durumunu "Cancelled" yap.(5) cancelCurrentGoal
# resume current operation -> go_to_goal'ı yeniden cagir(1) ilgili orderID'nin durumunu "OnItsWay" yap(2). okudugun goal ile (yani listenin en basindaki goal), bu goal tamamlanınca goals collection'ınını dinlemeye basla(2) resumeCurrentGoal



# Check if orderStatus is 'Cancelled' or 'Delivered'
#if change.document.to_dict().get("orderStatus") in ["Cancelled", "Delivered"]:
#print("Removing document...")
#change.document.reference.delete()



