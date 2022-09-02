from subprocess import check_output


def categories_file_to_class_names(categories_file_path):
    with open(categories_file_path, 'r') as f:
        class_names = [e.strip() for e in f.readlines()]
    return class_names


def get_rpi_ip():
    cmd = "hostname -I | awk '{print \"\"$1\"\"}'"
    return check_output(cmd, shell=True).decode('utf-8').rstrip()

def get_rpi_vpn_ip():
    cmd = "hostname -I | awk '{print \"\"$2\"\"}'"
    return check_output(cmd, shell=True).decode('utf-8').rstrip()



def ensure_loop_rate(rate, loop_time):
    """ Waits a bit to ensure a loop rate
    i.e  ensure_loop_rate(rate=1/fps, loop_time=time.time() - start_time)

    Parameters
    ----------
    rate : float
        Loop rate in Hertz.
    loop_time : float
        The time that the loop took until its end.
    """
    time.sleep(max(0, rate - loop_time))
