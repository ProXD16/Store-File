import dearpygui.dearpygui as dpg

def save_callback():
    print("Save Clicked")

def close_callback():
    print("Close Clicked")

dpg.create_context()

with dpg.window(label="Tutorial"):
    dpg.add_text("Hello, world")
    dpg.add_button(label="Save", callback=save_callback)
    dpg.add_input_text(label="string", default_value="Quick brown fox")
    dpg.add_slider_int(label="slider", default_value=0)

    with dpg.menu_bar():
        with dpg.menu(label="File"):
            dpg.add_menu_item(label="Save", callback=save_callback)
            dpg.add_menu_item(label="Close", callback=close_callback)

dpg.create_viewport(title='Custom Title', width=600, height=200)
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()