#!/usr/bin/env bash

#set log info color in terminal
BOLD='\033[1m'
RED='\033[0;31m'
GREEN='\033[32m'
WHITE='\033[34m'
YELLOW='\033[33m'
NO_COLOR='\033[0m'

if [[ "$(uname -m)" == "x86_64" ]]; then
    export SUPPORTED_NVIDIA_SMS="5.2 6.0 6.1 7.0 7.5 8.0 8.6"
else # AArch64
    export SUPPORTED_NVIDIA_SMS="5.3 6.2 7.2"
fi



function py3_version() {
    local version
    # python version is 3.9.5
    version="$(python3.9 --version | awk '{print $2}')"
    info "python version is ${version%.*}"
}

#information log in terminal
function info() {
    (>&2 echo -e "[${WHITE}${BOLD}INFO${NO_COLOR}] $*")
}

#error log in terminal
function error() {
    (>&2 echo -e "[${RED}ERROR${NO_COLOR}] $*")
}

#warning log in terminal
function warning() {
    (>&2 echo -e "${YELLOW}[WARNING] $*${NO_COLOR}")
}

#ok log in terminal
function ok() {
    (>&2 echo -e "[${GREEN}${BOLD} OK ${NO_COLOR}] $*")
}

function package_schema {
    local __link=$1
    local schema="http"

    if [[ "${__link##*.}" == "git" ]] ; then
        schema="git"
        echo $schema
        return
    fi

    IFS='.' # dot(.) is set as delimiter

    local __pkgname=$2
    read -ra __arr <<< "$__pkgname" # Array of tokens separated by IFS
    if [[ ${#__arr[@]} -gt 3 ]] && [[ "${__arr[-3]}" == "git" ]] \
        && [[ ${#__arr[-2]} -eq 7 ]] ; then
        schema="git"
    fi
    IFS=' ' # reset to default value after usage

    echo "$schema"
}

function _local_http_cached() {
    #-s Silent mode. Don't output anything
    #-f Fail silently (no output at all) on HTTP errors (H)
    #-I Show document info only  
    if /usr/bin/curl -sfI "${LOCAL_HTTP_ADDR}/$1" 
    then
        return
    fi
    false
}

function _checksum_check_pass() {
    local pkg="$1"
    local expected_cs="$2"
    # sha256sum was provided by coreutils
    local actual_cs=$(/usr/bin/sha256sum "${pkg}" | awk '{print $1}')
    if [[ "${actual_cs}" == "${expected_cs}" ]]; then
        true
    else
        warning "$(basename ${pkg}): checksum mismatch, ${expected_cs}" \
                "exected, got: ${actual_cs}"
        false
    fi
}

function download_if_not_cached {
    #this function have three parameters
    local pkg_name="$1"
    local expected_cs="$2"
    local url="$3"
    local download_adress="$4"
    #write download log information to /apollo_v7/tools/ApolloBuildEnv/build.log
    echo -e "apollo thirdparty ${pkg_name}\t${expected_cs}\t${url}" >> "${DOWNLOAD_LOG}"
    
    if _local_http_cached "${pkg_name}" ; then
        #if _local_http_cached function meet the conditions, run this branch
        local local_addr="${LOCAL_HTTP_ADDR}/${pkg_name}"
        echo ${LOCAL_HTTP_ADDR}
        info "Local http cache hit ${pkg_name}..."
        wget "${local_addr}" -O "$download_adress"/"$pkg_name"
        if _checksum_check_pass "${pkg_name}" "${expected_cs}"; then
            ok "Successfully downloaded ${pkg_name} from ${LOCAL_HTTP_ADDR}," \
               "will use it."
            return
        else
            warning "Found ${pkg_name} in local http cache, but checksum mismatch."
            rm -f "${pkg_name}"
        fi
    fi # end http cache check

    local my_schema
    #my_schema is http
    my_schema=$(package_schema "$url" "$pkg_name")
    #match the "http" condition
    if [[ "$my_schema" == "http" ]]; then
        info "Start to download $pkg_name from ${url} ..."
        #use wget to download opencv package
        wget  "$url" -O "$download_adress"/"$pkg_name" 
        ok "Successfully downloaded $pkg_name"
    elif [[ "$my_schema" == "git" ]]; then
        info "Clone into git repo $url..."
        #use git command to download opencv package
        git clone  "${url}" --branch master --recurse-submodules --single-branch ${download_adress}
        ok "Successfully cloned git repo: $url"
    else
        error "Unknown schema for package \"$pkg_name\", url=\"$url\""
    fi
}
