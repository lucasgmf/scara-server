import subprocess

if __name__ == "__main__":
    try:
        print("Starting Flask server...")
        subprocess.run(["python", "web_page.py"], check=True)
    except KeyboardInterrupt:
        print("\nServer stopped manually.")
    except Exception as e:
        print(f"Error: {e}")
