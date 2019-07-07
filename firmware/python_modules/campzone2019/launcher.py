import sys, uos as os, time, ujson
import machine, system, term_menu, virtualtimers, tasks.powermanagement as pm, buttons, defines
import rgb

# Application list

apps = []
current_index = 0


def show_text(text):
    # rgb.background((0, 50, 40))
    rgb.scrolltext(text, (80, 80, 0))


def clear():
    rgb.clear()


def add_app(app, information):
    global apps
    try:
        title = information["name"]
    except:
        title = app
    try:
        category = information["category"]
    except:
        category = ""
    info = {"file": app, "title": title, "category": category}
    apps.append(info)


def populate_apps():
    global apps
    apps = []
    try:
        userApps = os.listdir('lib')
    except OSError:
        userApps = []
    for app in userApps:
        add_app(app, read_metadata(app))
    add_app("snake", {"name": "Snake", "category": "system"})
    add_app("installer", {"name": "Installer", "category": "system"})
    add_app("update", {"name": "Update apps", "category": "system"})
    add_app("checkforupdates", {"name": "Update firmware", "category": "system"})
    add_app(machine.nvs_getstr("system", 'default_app'), {"name": "Home", "category": "system"})


def render_current_app():
    clear()
    app = apps[current_index]
    show_text(app["title"])


# Read app metadata
def read_metadata(app):
    try:
        install_path = get_install_path()
        info_file = "%s/%s/metadata.json" % (install_path, app)
        print("Reading " + info_file + "...")
        with open(info_file) as f:
            information = f.read()
        return ujson.loads(information)
    except BaseException as e:
        print("[ERROR] Can not read metadata for app " + app)
        sys.print_exception(e)
        information = {"name": app, "description": "", "category": "", "author": "", "revision": 0}
        return [app, ""]


# Uninstaller

def uninstall():
    global options
    selected = options.selected_index()
    options.destroy()

    global currentListTitles
    global currentListTargets

    if currentListTargets[selected]["category"] == "system":
        # dialogs.notice("System apps can not be removed!","Can not uninstall '"+currentListTitles[selected]+"'")
        show_text("System apps can not be removed!")
        time.sleep(2)
        start()
        return

    def perform_uninstall(ok):
        global install_path
        if ok:
            show_text("Removing " + currentListTitles[selected] + "...")
            install_path = get_install_path()
            for rm_file in os.listdir("%s/%s" % (install_path, currentListTargets[selected]["file"])):
                show_text("Deleting '" + rm_file + "'...")
                os.remove("%s/%s/%s" % (install_path, currentListTargets[selected]["file"], rm_file))
            show_text("Deleting folder...")
            os.rmdir("%s/%s" % (install_path, currentListTargets[selected]["file"]))
            show_text("Uninstall completed!")
        start()

    # TODO: add removal dialog again
    # dialogs.prompt_boolean('Remove %s?' % currentListTitles[selected], cb=perform_uninstall)


# Run app

def run():
    system.start(apps[current_index]["file"], status=True)


# Path

def expandhome(s):
    if "~/" in s:
        h = os.getenv("HOME")
        s = s.replace("~/", h + "/")
    return s


def get_install_path():
    global install_path
    if install_path is None:
        # sys.path[0] is current module's path
        install_path = sys.path[1]
    install_path = expandhome(install_path)
    return install_path


# Actions        
def input_run(pressed):
    pm.feed()
    if pressed:
        run()


def input_uninstall(pressed):
    pm.feed()
    if pressed:
        uninstall()


def input_up(pressed):
    global current_index

    pm.feed()
    if pressed:
        current_index = (current_index - 1) % len(apps)
        render_current_app()


def input_down(pressed):
    global current_index

    pm.feed()
    if pressed:
        current_index = (current_index + 1) % len(apps)
        render_current_app()


def input_other(pressed):
    pm.feed()


# Power management
def pm_cb(dummy):
    appglue.home()


def init_power_management():
    virtualtimers.activate(1000)  # Start scheduler with 1 second ticks
    pm.set_timeout(5 * 60 * 1000)  # Set timeout to 5 minutes
    pm.callback(pm_cb)  # Go to splash instead of sleep
    pm.feed()  # Feed the power management task, starts the countdown...


# Main application
def start():
    global options
    global install_path
    options = None
    install_path = None

    buttons.register(defines.BTN_A, input_run)
    buttons.register(defines.BTN_UP, input_up)
    buttons.register(defines.BTN_DOWN, input_down)
    buttons.register(defines.BTN_LEFT, input_other)
    buttons.register(defines.BTN_RIGHT, input_other)

    populate_apps()
    render_current_app()


start()
init_power_management()
menu = term_menu.UartMenu(None, pm)
menu.main()