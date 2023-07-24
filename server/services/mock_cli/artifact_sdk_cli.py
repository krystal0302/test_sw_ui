#!/usr/bin/python

import os
import shutil
import argparse
import json
import re
import subprocess
import requests
import yaml
import tarfile

from distutils.dir_util import copy_tree

VERSION_STR="0.9.2"

absPath = os.path.abspath(__file__)
cwd = os.path.dirname(absPath)+'/'

class ArtifactWrappers():
    def __init__(self):
        self.wrapper_list = os.listdir(cwd+"wrappers")
        self.wrapper_list.remove(".gitignore")
        # TODO: fail safe, if the file not in the list
        # self.wrapper_list.remove("COLCON_IGNORE")
        self.type_list = os.listdir(cwd+"types")
        self.type_list.remove(".gitkeep")
    
    def listTypes(self):
        return self.type_list

    def listWrappers(self):
        wrapper_list = []
        for w in self.wrapper_list:
            wrapper_dict = {}
            # Get service name
            json_filename = [f for f in os.listdir(cwd+"wrappers/"+w) if f.endswith(".json")][0]
            json_file = open(cwd+"wrappers/"+w+"/"+json_filename)
            json_dict = json.load(json_file)
            json_file.close()
            # Get build datetime
            if os.path.isdir(cwd+"wrappers/"+w+"/log"):
                build_log_time = [f for f in os.listdir(cwd+"wrappers/"+w+"/log") if f.startswith("build_")][0][6:]
            else:
                build_log_time = "Not built yet"
            # Save to dictionary
            wrapper_dict["name"] = w
            wrapper_dict["type"] = json_filename[:-5] # Remove .json
            wrapper_dict["services"] = list(json_dict["services"].keys())
            wrapper_dict["timestamp"] = build_log_time
            wrapper_list.append(wrapper_dict)
        return wrapper_list
    
    def createWrapper(self, wrapper_name, type_name):
        if wrapper_name in self.wrapper_list:
            return {"status":"error", "output":"Duplicate wrapper name"}
        if type_name not in self.type_list:
            return {"status":"error", "output":"No such wrapper type"}
        os.mkdir(cwd+"wrappers/"+wrapper_name)
        os.mkdir(cwd+"wrappers/"+wrapper_name+"/src")
        copy_tree(cwd+"types/"+type_name+"/"+type_name+"_wrapper", cwd+"wrappers/"+wrapper_name+"/src")
        shutil.copy(cwd+"types/"+type_name+"/"+type_name+".json", cwd+"wrappers/"+wrapper_name)
        self.wrapper_list.append(wrapper_name)
        return {"status":"success", "output":"Success"}
    
    def deleteWrapper(self, wrapper_name):
        if wrapper_name not in self.wrapper_list:
            return {"status":"error", "output":"No such wrapper name"}
        shutil.rmtree(cwd+"wrappers/"+wrapper_name)
        self.wrapper_list.remove(wrapper_name)
        return {"status":"success", "output":"Success"}

    def buildWrapper(self, wrapper_name):
        if wrapper_name not in self.wrapper_list:
            return {"status":"error", "output":"No such wrapper name"}
        process_runner = subprocess.run("cd "+ cwd + "wrappers/"+wrapper_name+" && " +
                                        "rm -rf build install log && " +
                                        "colcon build",
                                        stdout=subprocess.DEVNULL,
                                        stderr=subprocess.DEVNULL,
                                        shell=True)
        return {"status":"success", "output":"Success"}
    
    def getServiceCode(self, wrapper_name, service_name):
        if wrapper_name not in self.wrapper_list:
            return {"status":"error", "output":"No such wrapper name"}
        typename = [f for f in os.listdir(cwd+"wrappers/"+wrapper_name) if f.endswith(".json")][0].split(".")[0]
        # Open user_wrapper.py
        user_filename = cwd+"wrappers/"+wrapper_name+"/src/"+typename+"_wrapper/user_wrapper.py"
        service_content = ""
        with open(user_filename) as user_file:
            file_content = user_file.read()
            if "# "+service_name+":start" not in file_content:
                return {"status":"error", "output":"No such service name"}
            # Get the content between `# service_name:start` and `# service_name:stop`
            service_content = re.search(r"# "+service_name+":start\n([\s\S]*?)# "+service_name+":stop\n", file_content).group(1)
        return {"status":"success", "output":service_content}
    
    def saveServiceCode(self, wrapper_name, service_name, python_code):
        if wrapper_name not in self.wrapper_list:
            return {"status":"error", "output":"No such wrapper name"}
        typename = [f for f in os.listdir(cwd+"wrappers/"+wrapper_name) if f.endswith(".json")][0].split(".")[0]
        # Filter forbidden python code
        filter_list = ["import", "# "+wrapper_name+":start", "# "+wrapper_name+":stop"]
        for f in filter_list:
            if f in python_code:
                return {"status":"error", "output":"No supported Python code"}
        # Open user_wrapper.py
        user_filename = cwd+"wrappers/"+wrapper_name+"/src/"+typename+"_wrapper/user_wrapper.py"
        new_content = ""
        with open(user_filename) as user_file:
            file_content = user_file.read()
            if "# "+service_name+":start" not in file_content:
                return {"status":"error", "output":"No such service name"}
            # Update the content between `# service_name:start` and `# service_name:stop`
            new_content = re.sub(r"(# "+service_name+":start\n)[\s\S]*?(# "+service_name+":stop\n)", r"\1"+python_code+r"\2", file_content)
        # Update the file
        with open(user_filename, "w") as user_file:
            user_file.write(new_content)
        return {"status":"success", "output":"Success"}

    def getWrapperCode(self, wrapper_name):
        if wrapper_name not in self.wrapper_list:
            return None
        for path, subdirs, files in os.walk(cwd+"wrappers/"+wrapper_name+"/src"):
            for f in files:
                if f == "user_wrapper.py":
                    wrapper_code_path = os.path.join(path, f)
                    break
        wrapper_code = ""
        with open(wrapper_code_path) as user_file:
            wrapper_code = user_file.read()
        return wrapper_code

class ArtifactStatus():
    def __init__(self, username="root", password="root@farobot", url="http://127.0.0.1:5000"):
        post_data = {"grant_type": "password", "username": username, "password": password}
        self.url = url
        r = requests.post(self.url+"/login/access-token", data=post_data)
        if r.status_code == 200:
            self.auth = r.json()['access_token']
        else:
            self.auth = "Not auth"

    def listArtifact(self):
        header_data = {"Authorization": "Bearer "+self.auth}
        r = requests.get(self.url+"/artifacts/scan", headers=header_data)
        if r.status_code == 200:
            return r.json()['artifacts']
        else:
            return []

    def deployArtifact(self, device_id, wrapper_name):
        wrapper_path = cwd+"wrappers/"+wrapper_name
        return_json = {}
        if not os.path.isdir(wrapper_path):
            return_json["status"] = "error"
            return_json["output"] = "No such wrapper name"
            return return_json
        with tarfile.open("user_wrapper.tar.gz", "w:gz") as tar:
            tar.add(wrapper_path, arcname=os.path.basename(wrapper_path))
        header_data = {"Authorization": "Bearer "+self.auth}
        post_data = {"artifact_list": [device_id]}
        r = requests.post(self.url+"/artifacts/deploy", headers=header_data, json=post_data)
        if r.status_code == 201:
            return_json["status"] = "success"
            return_json["output"] = "Success"
        else:
            return_json["status"] = "error"
            return_json["output"] = "No such device ID"
        return return_json

    def runArtifact(self, device_id, wrapper_name, service_name):
        artifact_wrapper = ArtifactWrappers()
        return_json = {}
        wrapper_code = artifact_wrapper.getWrapperCode(wrapper_name)
        if wrapper_code == None:
            return_json["status"] = "error"
            return_json["output"] = "No such wrapper name"
            return return_json
        header_data = {"Authorization": "Bearer "+self.auth}
        yaml_dict = {"service": service_name, "code": wrapper_code}
        post_data = {"artifact_list": [device_id], "artifact_code": yaml.dump(yaml_dict)}
        r = requests.put(self.url+"/artifacts/run_artifact", headers=header_data, json=post_data)
        if r.status_code == 200:
            return_json["status"] = "success"
            return_json["output"] = "Success"
        else:
            return_json["status"] = "error"
            return_json["output"] = "No such device ID"
        return return_json

    def runlogArtifact(self, device_id):
        header_data = {"Authorization": "Bearer "+self.auth}
        post_data = {"artifact_list": [device_id]}
        r = requests.post(self.url+"/artifacts/get_runlog", headers=header_data, json=post_data)
        return_json = {}
        if r.status_code == 200:
            runlog_yaml = yaml.safe_load(r.json()['artifact_runlog'][0]['run_log'])
            return_json["status"] = runlog_yaml["status"]
            if runlog_yaml["status"] == "success":
                return_json["output"] = runlog_yaml["log"]
            elif runlog_yaml["status"] == "running":
                return_json["output"] = "In Progress"
            elif runlog_yaml["status"] == "error":
                return_json["output"] = "No running log"
        else:
            return_json["status"] == "error"
            return_json["output"] = "No such device ID"
        return return_json

def cmd_parser():
    main_parser = argparse.ArgumentParser(description="Artifact SDK CLI tool")
    main_parser.add_argument('-v', '--version', action='version', version=VERSION_STR)
    cmd_parser = main_parser.add_subparsers(dest='cmd', metavar='cmd', required=True, help="Functions")

    # type
    type_parser = cmd_parser.add_parser('type', help="Related to artifact type")
    type_subparser = type_parser.add_subparsers(dest='subcmd', metavar='subcmd', required=True, help="Functions")
    ## list: python3 artifact_sdk_cli.py type list
    typelist_parsrer = type_subparser.add_parser('list', help="list artifact type")
    ## create
    # TODO: type create

    # wrapper
    wrapper_parser = cmd_parser.add_parser('wrapper', help="Related to wrapper")
    wrapper_subparser = wrapper_parser.add_subparsers(dest='subcmd', metavar='subcmd', required=True, help="Functions")
    ## list: python3 artifact_sdk_cli.py wrapper list
    wrapperlist_parser = wrapper_subparser.add_parser('list', help="list wrappers")
    ## create: python3 artifact_sdk_cli.py wrapper create my_wrapper type
    wrappercreate_parser = wrapper_subparser.add_parser('create', help="create wrappers")
    wrappercreate_parser.add_argument('wrapper_name', metavar='wrapper_name', help="Your wrapper name")
    wrappercreate_parser.add_argument('wrapper_type', metavar='wrapper_type', help="Your wrapper type")
    ## delete: python3 artifact_sdk_cli.py wrapper delete my_wrapper
    wrapperdelete_parser = wrapper_subparser.add_parser('delete', help="delete wrappers")
    wrapperdelete_parser.add_argument('wrapper_name', metavar='wrapper_name', help="Your wrapper name")
    ## get_service: python3 artifact_sdk_cli.py wrapper get_service my_wrapper my_service
    wrappergetsrv_parser = wrapper_subparser.add_parser('get_service', help="get service code")
    wrappergetsrv_parser.add_argument('wrapper_name', metavar='wrapper_name', help="Your wrapper name")
    wrappergetsrv_parser.add_argument('service_name', metavar='service_name', help="Your service name")
    ## save_service: python3 artifact_sdk_cli.py wrapper save_service my_wrapper my_service python_code
    wrappersavesrv_parser = wrapper_subparser.add_parser('save_service', help="save service code")
    wrappersavesrv_parser.add_argument('wrapper_name', metavar='wrapper_name', help="Your wrapper name")
    wrappersavesrv_parser.add_argument('service_name', metavar='service_name', help="Your service name")
    wrappersavesrv_parser.add_argument('python_code', metavar='python_code', help="Your python code")
    ## build
    wrapperbuild_parser = wrapper_subparser.add_parser('build', help="build wrapper")
    wrapperbuild_parser.add_argument('wrapper_name', metavar='wrapper_name', help="Your wrapper name")

    # artifact
    artifact_parser = cmd_parser.add_parser('artifact', help="Related to artifact")
    artifact_subparser = artifact_parser.add_subparsers(dest='subcmd', metavar='subcmd', required=True, help="Functions")
    ## list
    artifactlist_parser = artifact_subparser.add_parser('list', help="list artifacts")
    ## deploy
    artifactdeploy_parser = artifact_subparser.add_parser('deploy', help="deploy artifact")
    artifactdeploy_parser.add_argument('device_id', metavar='device_id', help="Your device ID")
    artifactdeploy_parser.add_argument('wrapper_name', metavar='wrapper_name', help="Your wrapper name")
    ## run
    artifactrun_parser = artifact_subparser.add_parser('run', help="run artifact service")
    artifactrun_parser.add_argument('device_id', metavar='device_id', help="Your device ID")
    artifactrun_parser.add_argument('wrapper_name', metavar='wrapper_name', help="Your wrapper name")
    artifactrun_parser.add_argument('service_name', metavar='service_name', help="Your service name")
    ## run_log
    artifactrunlog_parser = artifact_subparser.add_parser('run_log', help="get artifact service log")
    artifactrunlog_parser.add_argument('device_id', metavar='device_id', help="Your device ID")

    args = main_parser.parse_args()
    return args

def main():
    args = cmd_parser()

    if args.cmd == "type":
        artifact_wrapper = ArtifactWrappers()
        if args.subcmd == "list":
            print(json.dumps(artifact_wrapper.listTypes()))
    elif args.cmd == "wrapper":
        artifact_wrapper = ArtifactWrappers()
        if args.subcmd == "list":
            print(json.dumps(artifact_wrapper.listWrappers()))
        elif args.subcmd == "create":
            print(json.dumps(artifact_wrapper.createWrapper(args.wrapper_name, args.wrapper_type)))
        elif args.subcmd == "delete":
            print(json.dumps(artifact_wrapper.deleteWrapper(args.wrapper_name)))
        elif args.subcmd == "get_service":
            print(json.dumps(artifact_wrapper.getServiceCode(args.wrapper_name, args.service_name)))
        elif args.subcmd == "save_service":
            print(json.dumps(artifact_wrapper.saveServiceCode(args.wrapper_name, args.service_name, args.python_code)))
        elif args.subcmd == "build":
            print(json.dumps(artifact_wrapper.buildWrapper(args.wrapper_name)))
    elif args.cmd == "artifact":
        artifact = ArtifactStatus()
        if args.subcmd == "list":
            print(json.dumps(artifact.listArtifact()))
        elif args.subcmd == "deploy":
            print(json.dumps(artifact.deployArtifact(args.device_id, args.wrapper_name)))
        elif args.subcmd == "run":
            print(json.dumps(artifact.runArtifact(args.device_id, args.wrapper_name, args.service_name)))
        elif args.subcmd == "run_log":
            print(json.dumps(artifact.runlogArtifact(args.device_id)))

if __name__ == '__main__':
    main()