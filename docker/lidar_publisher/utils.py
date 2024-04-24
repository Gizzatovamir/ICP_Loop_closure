import sqlite3
from typing import Tuple

def connect(sqlite_file):
    """Make connection to an SQLite database file."""
    conn = sqlite3.connect(sqlite_file)
    c = conn.cursor()
    return conn, c


def close(conn):
    """Close connection to the database."""
    conn.close()


def countRows(cursor, table_name, print_out=False):
    """Returns the total number of rows in the database."""
    cursor.execute("SELECT COUNT(*) FROM {}".format(table_name))
    count = cursor.fetchall()
    if print_out:
        print("\nTotal rows: {}".format(count[0][0]))
    return count[0][0]


def getHeaders(cursor, table_name, print_out=False):
    """Returns a list of tuples with column informations:
    (id, name, type, notnull, default_value, primary_key)
    """
    # Get headers from table "table_name"
    cursor.execute("PRAGMA TABLE_INFO({})".format(table_name))
    info = cursor.fetchall()
    if print_out:
        print("\nColumn Info:\nID, Name, Type, NotNull, DefaultVal, PrimaryKey")
        for col in info:
            print(col)
    return info


def getAllElements(cursor, table_name, print_out=False):
    """Returns a dictionary with all elements of the table database."""
    # Get elements from table "table_name"
    cursor.execute("SELECT * from({}) LIMIT 1000".format(table_name))
    records = cursor.fetchall()
    if print_out:
        print("\nAll elements:")
        for row in records:
            print(row)
    return records


def get_topic_id(cursor: sqlite3.Cursor, topic_name: str) -> int:
    return int(
        cursor.execute(f"SELECT id FROM topics WHERE name == '{topic_name}'").fetchone()[
            0
        ]
    )


def getSingleMessageInTopic(cursor: sqlite3.Cursor, topic_name: str, timestamp) -> Tuple[str, any]:
    topic_id = get_topic_id(cursor, topic_name)
    timestamp, data = cursor.execute(
        f"SELECT timestamp, data FROM messages WHERE topic_id == {topic_id} and timestamp > {timestamp}"
    ).fetchone()
    return str(timestamp), data


def isTopic(cursor, topic_name, print_out=False):
    """Returns topic_name header if it exists. If it doesn't, returns empty.
    It returns the last topic found with this name.
    """
    boolIsTopic = False
    topicFound = []

    # Get all records for 'topics'
    records = getAllElements(cursor, "topics", print_out=False)

    # Look for specific 'topic_name' in 'records'
    for row in records:
        if row[1] == topic_name:  # 1 is 'name' TODO
            boolIsTopic = True
            topicFound = row
    if print_out:
        if boolIsTopic:
            # 1 is 'name', 0 is 'id' TODO
            print("\nTopic named", topicFound[1], " exists at id ", topicFound[0], "\n")
        else:
            print("\nTopic", topic_name, "could not be found. \n")

    return topicFound


def getAllMessagesInTopic(cursor, topic_name, print_out=False):
    """Returns all timestamps and messages at that topic.
    There is no deserialization for the BLOB data.
    """
    count = 0
    timestamps = []
    messages = []

    # Find if topic exists and its id
    topicFound = isTopic(cursor, topic_name, print_out=False)

    # If not find return empty
    if not topicFound:
        print("Topic", topic_name, "could not be found. \n")
    else:
        records = getAllElements(cursor, "messages", print_out=False)

        # Look for message with the same id from the topic
        for row in records:
            if row[1] == topicFound[0]:  # 1 and 0 is 'topic_id' TODO
                count = count + 1  # count messages for this topic
                timestamps.append(row[2])  # 2 is for timestamp TODO
                messages.append(row[3])  # 3 is for all messages

        # Print
        if print_out:
            print("\nThere are ", count, "messages in ", topicFound[1])

    return timestamps, messages
