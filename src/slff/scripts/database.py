#!/usr/bin/python3

import mysql.connector
import database_sql as db_sql
import database_fn as db_fn

import rospy
from slff.msg import rfid_tag
from slff.msg import gto_present
from slff.msg import gto_notification
from slff.msg import gto_store


localDB = None
localDBCursor = None

# =============================================================================
# -----------------------------------------------------------------------------
# =============================================================================


def cllbck_tim_1hz(event):
    if database_routine() == -1:
        rospy.signal_shutdown("signal shutdown")

# =============================================================================
# -----------------------------------------------------------------------------
# =============================================================================


def cllbck_sub_rfid_tag(msg):
    db_connect()
    db_fn.replace_tbl_rfid_tag(localDB, localDBCursor, msg)
    db_close()


def cllbck_sub_gto_present(msg):
    db_connect()
    db_fn.replace_tbl_gto_present(localDB, localDBCursor, msg)
    db_close()


def cllbck_sub_gto_notification(msg):
    db_connect()
    db_fn.replace_tbl_gto_notification(localDB, localDBCursor, msg)
    db_close()


def cllbck_sub_gto_store(msg):
    db_connect()
    db_fn.replace_tbl_gto_store(localDB, localDBCursor, msg)
    db_close()

# =============================================================================
# -----------------------------------------------------------------------------
# =============================================================================


def db_connect():
    # Try connect to database
    try:
        global localDB
        global localDBCursor
        localDB = mysql.connector.connect(
            host=database_host,
            user=database_user,
            password=database_password,
            database=database_name
        )
        localDBCursor = localDB.cursor()
    # Catch and display error message
    except:
        print('')
        print(mysql.connector.Error)
        print('')

        return -1

    return 0


def db_close():
    global localDB
    localDB.close()

# =============================================================================
# -----------------------------------------------------------------------------
# =============================================================================


def database_init():
    global localDB
    global localDBCursor

    if db_connect() == -1:
        return -1
    localDBCursor.execute(db_sql.sql_create_tbl_rfid_tag)
    localDBCursor.execute(db_sql.sql_create_tbl_gto_present)
    localDBCursor.execute(db_sql.sql_create_tbl_gto_notification)
    localDBCursor.execute(db_sql.sql_create_tbl_gto_store)
    localDBCursor.execute(db_sql.sql_create_view_transaction)
    db_close()

    return 0


def database_routine():

    return 0

# =============================================================================
# -----------------------------------------------------------------------------
# =============================================================================


if __name__ == '__main__':
    # Parameter
    global database_host
    global database_name
    global database_user
    global database_password
    # Timer
    global tim_1hz
    # Subscriber
    global sub_rfid_tag
    global sub_gto_present
    global sub_gto_notification
    global sub_gto_store

    rospy.init_node('database')

    # Parameter
    database_host = rospy.get_param('database/host', 'localhost')
    database_name = rospy.get_param('database/name', 'slff')
    database_user = rospy.get_param('database/user', 'slff')
    database_password = rospy.get_param('database/password', 'slff')
    # Timer
    tim_1hz = rospy.Timer(rospy.Duration(1), cllbck_tim_1hz)
    # Subscriber
    sub_rfid_tag = rospy.Subscriber('rfid/tag', rfid_tag, cllbck_sub_rfid_tag)
    sub_gto_present = rospy.Subscriber('gto/present', gto_present, cllbck_sub_gto_present)
    sub_gto_notification = rospy.Subscriber('gto/notification', gto_notification, cllbck_sub_gto_notification)
    sub_gto_store = rospy.Subscriber('gto/store', gto_store, cllbck_sub_gto_store)

    if database_init() == -1:
        rospy.signal_shutdown("signal shutdown")

    rospy.spin()
