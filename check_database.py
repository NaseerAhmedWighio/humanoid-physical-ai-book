import sqlite3
from datetime import datetime
import hashlib

def connect_db():
    """Connect to the SQLite database"""
    # First try the backend directory, then root, then with different names
    import os
    possible_paths = [
        'backend/humanoid_ai_book.db',
        'backend/database.db',
        'backend/users.db',
        'humanoid_ai_book.db',
        'database.db',
        'users.db',
        '../backend/humanoid_ai_book.db'
    ]

    conn = None
    for path in possible_paths:
        if os.path.exists(path):
            conn = sqlite3.connect(path)
            print(f"Connected to database: {path}")
            break

    if conn is None:
        # Create a new connection to a default database
        conn = sqlite3.connect('humanoid_ai_book.db')
        print("Created new database: humanoid_ai_book.db")

    conn.row_factory = sqlite3.Row  # This allows us to access columns by name
    return conn

def view_users():
    """View all users in the database"""
    conn = connect_db()
    cursor = conn.cursor()

    try:
        # Check if the users table exists
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='users';")
        table_exists = cursor.fetchone()

        if not table_exists:
            print("No 'users' table found in the database.")
            # Let's check what tables exist
            cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
            tables = cursor.fetchall()
            if tables:
                print("Available tables:")
                for table in tables:
                    print(f"  - {table[0]}")
            else:
                print("No tables found in the database.")
            return

        # Try to get all available columns in the users table
        cursor.execute("PRAGMA table_info(users)")
        columns_info = cursor.fetchall()
        available_columns = [col[1] for col in columns_info]

        # Build dynamic query based on available columns
        # Only select the columns that actually exist
        columns_to_select = []
        for col in ['id', 'email', 'hashed_password', 'has_mobile', 'has_laptop', 'has_physical_robot',
                   'has_other_hardware', 'web_dev_experience', 'language_preference',
                   'personalization_enabled', 'created_at', 'updated_at']:
            if col in available_columns:
                columns_to_select.append(col)

        if not columns_to_select:
            print("No expected columns found in the users table.")
            return

        query = f"SELECT {', '.join(columns_to_select)} FROM users ORDER BY created_at DESC"
        cursor.execute(query)
        users = cursor.fetchall()

        if users:
            print(f"\n{'='*100}")
            print(f"{'ID':<36} | {'EMAIL':<30} | CREATED AT")
            print(f"{'='*100}")

            for user in users:
                # Safely access fields that might not exist
                user_id = user['id'] if 'id' in available_columns else 'N/A'
                email = user['email'] if 'email' in available_columns else 'N/A'
                created_at = user['created_at'] if 'created_at' in available_columns else 'N/A'

                print(f"{user_id:<36} | {email:<30} | {created_at}")

            print(f"\nDetailed information:")
            print(f"{'='*120}")
            for i, user in enumerate(users, 1):
                print(f"\n{i}. User ID: {user['id'] if 'id' in available_columns else 'N/A'}")
                print(f"   Email: {user['email'] if 'email' in available_columns else 'N/A'}")
                print(f"   Password (hashed): {user['hashed_password'] if 'hashed_password' in available_columns else 'N/A'}")
                print(f"   Mobile: {user['has_mobile'] if 'has_mobile' in available_columns else 'N/A'}")
                print(f"   Laptop: {user['has_laptop'] if 'has_laptop' in available_columns else 'N/A'}")
                print(f"   Physical Robot: {user['has_physical_robot'] if 'has_physical_robot' in available_columns else 'N/A'}")
                print(f"   Other Hardware: {user['has_other_hardware'] if 'has_other_hardware' in available_columns else 'N/A'}")
                print(f"   Web Experience: {user['web_dev_experience'] if 'web_dev_experience' in available_columns else 'N/A'}")
                print(f"   Language Preference: {user['language_preference'] if 'language_preference' in available_columns else 'N/A'}")
                print(f"   Personalization Enabled: {user['personalization_enabled'] if 'personalization_enabled' in available_columns else 'N/A'}")
                print(f"   Created: {user['created_at'] if 'created_at' in available_columns else 'N/A'}")
                print(f"   Updated: {user['updated_at'] if 'updated_at' in available_columns else 'N/A'}")
        else:
            print("No users found in the database.")

    except sqlite3.Error as e:
        print(f"Database error viewing users: {e}")
    except Exception as e:
        print(f"Error viewing users: {e}")
    finally:
        conn.close()

def delete_user_by_email(email):
    """Delete a user by email"""
    conn = connect_db()
    cursor = conn.cursor()

    try:
        # Check if the users table exists
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='users';")
        table_exists = cursor.fetchone()

        if not table_exists:
            print("No 'users' table found in the database.")
            return

        # First, check if user exists
        cursor.execute("SELECT id, email FROM users WHERE email = ?", (email,))
        user = cursor.fetchone()

        if user:
            print(f"Found user: {user['email']} (ID: {user['id']})")

            # Show the user's password hash for verification (optional)
            cursor.execute("SELECT email, hashed_password FROM users WHERE email = ?", (email,))
            user_with_password = cursor.fetchone()
            if user_with_password and 'hashed_password' in [col[1] for col in cursor.execute("PRAGMA table_info(users);").fetchall()]:
                print(f"Password hash: {user_with_password['hashed_password']}")

            confirm = input(f"\nAre you sure you want to delete this user? (yes/no): ")

            if confirm.lower() == 'yes':
                cursor.execute("DELETE FROM users WHERE email = ?", (email,))
                conn.commit()
                print(f"User {email} has been deleted successfully.")
            else:
                print("Deletion cancelled.")
        else:
            print(f"No user found with email: {email}")

    except sqlite3.Error as e:
        print(f"Database error deleting user: {e}")
    except Exception as e:
        print(f"Error deleting user: {e}")
    finally:
        conn.close()

def delete_user_by_id(user_id):
    """Delete a user by ID"""
    conn = connect_db()
    cursor = conn.cursor()

    try:
        # Check if the users table exists
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='users';")
        table_exists = cursor.fetchone()

        if not table_exists:
            print("No 'users' table found in the database.")
            return

        # First, check if user exists
        cursor.execute("SELECT id, email FROM users WHERE id = ?", (user_id,))
        user = cursor.fetchone()

        if user:
            print(f"Found user: {user['email']} (ID: {user['id']})")
            confirm = input(f"\nAre you sure you want to delete this user? (yes/no): ")

            if confirm.lower() == 'yes':
                cursor.execute("DELETE FROM users WHERE id = ?", (user_id,))
                conn.commit()
                print(f"User {user['email']} (ID: {user['id']}) has been deleted successfully.")
            else:
                print("Deletion cancelled.")
        else:
            print(f"No user found with ID: {user_id}")

    except sqlite3.Error as e:
        print(f"Database error deleting user: {e}")
    except Exception as e:
        print(f"Error deleting user: {e}")
    finally:
        conn.close()

def delete_all_users():
    """Delete all users (use with caution!)"""
    conn = connect_db()
    cursor = conn.cursor()

    try:
        # Check if the users table exists
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='users';")
        table_exists = cursor.fetchone()

        if not table_exists:
            print("No 'users' table found in the database.")
            return

        # Count users first
        cursor.execute("SELECT COUNT(*) as count FROM users")
        count_result = cursor.fetchone()
        if count_result:
            count = count_result['count']
        else:
            count = 0

        if count > 0:
            print(f"There are {count} users in the database.")
            confirm = input(f"\nAre you SURE you want to delete ALL users? This cannot be undone! (yes/no): ")

            if confirm.lower() == 'yes':
                cursor.execute("DELETE FROM users")
                conn.commit()
                print(f"All {count} users have been deleted successfully.")
            else:
                print("Deletion cancelled.")
        else:
            print("No users to delete.")

    except sqlite3.Error as e:
        print(f"Database error deleting all users: {e}")
    except Exception as e:
        print(f"Error deleting all users: {e}")
    finally:
        conn.close()

def main():
    print("Database Management Tool for Humanoid AI Book")
    print("="*50)

    while True:
        print("\nOptions:")
        print("1. View all users")
        print("2. Delete user by email")
        print("3. Delete user by ID")
        print("4. Delete all users (DANGER!)")
        print("5. Exit")

        choice = input("\nEnter your choice (1-5): ")

        if choice == '1':
            view_users()
        elif choice == '2':
            email = input("Enter email to delete: ")
            delete_user_by_email(email)
        elif choice == '3':
            user_id = input("Enter user ID to delete: ")
            delete_user_by_id(user_id)
        elif choice == '4':
            delete_all_users()
        elif choice == '5':
            print("Goodbye!")
            break
        else:
            print("Invalid choice. Please try again.")

if __name__ == "__main__":
    main()