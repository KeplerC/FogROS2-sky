
import sky 
import subprocess
from time import sleep
import os 
from .name_generator import get_unique_name


class SkyCluster():
    def __init__(self, config_path, logger) -> None:
        self.config_path = config_path
        self.logger = logger
        self._name = get_unique_name()
        
    def init_cluster(self):
        self.logger.info(f"Creating new Sky cluster {self._name}")
        with sky.Dag() as dag:
            t = sky.Task.from_yaml(self.config_path)

        if sky.status("sky-fogros") == []:
            # create a new cluster
            # sky.launch(dag, cluster_name = "sky-fogros", idle_minutes_to_autostop=100)
            pid = os.fork()
            if pid == 0:
                # child process, just launch the yaml cloud node
                sky.launch(dag, cluster_name = "sky-fogros", idle_minutes_to_autostop=100)
                exit(0)
        else:
            # run with the same cluster
            sky.exec(dag, cluster_name = "sky-fogros")
        self.wait_for_cluster()

    def init_spot_cluster(self):
        # run command sky spot launch -f /tmp/sky.yaml
        # p = subprocess.Popen("sky spot launch /tmp/sky.yaml", stdout=subprocess.PIPE, shell=True)
        pid = os.fork()
        if pid == 0:
            os.execvp("sky", ["sky", "spot", "launch", "--yes", "--detach-run", "/tmp/sky.yaml"])
        else:
            cluster_name = self.get_spot_cluster_name()
            self.wait_for_cluster(cluster_name)
    
    def get_spot_cluster_name(self):
        while True:
            try:
                for cluster in sky.status():
                    if cluster["name"].startswith("sky-spot-controller"):
                        return cluster["name"]
            except:
                sleep(1)

    def wait_for_cluster(self, cluster_name = "sky-fogros"):
        user = subprocess.check_output('whoami', shell=True).decode().strip()

        while (sky.status(cluster_name) == [] or str(sky.status(cluster_name)[0]["status"]) != "ClusterStatus.UP"):
            if sky.status(cluster_name) == []:
                self.logger.info("Waiting for the cluster to be created")
            else:
                self.logger.info(f"Cluster status: {str(sky.status(cluster_name)[0]['status'])}")
            sleep(1)

        # here we only need the ip address of the head node
        try:
            self._ip = sky.status["handle"].__dict__["stable_internal_external_ips"][0][1] 
            self._ssh_key_path = f"/home/{user}/.ssh/sky-key"
        except:
            # TODO: placeholders
            self._ip = None
            self._ssh_key_path = f"/home/{user}/.ssh/sky-key"
        self._is_created = True