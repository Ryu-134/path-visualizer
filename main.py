import tkinter as tk
from src.ui import GUI


def main():
    root = tk.Tk()
    root.title("Path Visualizer - Interactive Graph Algorithm Engine")
    root.geometry("1200x900") 
    try:
        root.state('zoomed') 
    except:
        try:
            root.attributes('-zoomed', True)
        except:
            pass
    
    app = GUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()