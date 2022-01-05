<p align="center">
    <img src="https://i.imgur.com/S8pm7jM.png" width="200" height="200">
</p>

**GitHub Stat(s):**  
![GitHub code size in bytes](https://img.shields.io/github/languages/code-size//tbriandy/SLFF-JMIOT?logo=github) ![GitHub all releases](https://img.shields.io/github/downloads//tbriandy/SLFF-JMIOT/total?logo=github) [![Build Status](https://img.shields.io/endpoint.svg?url=https%3A%2F%2Factions-badge.atrox.dev%2Fprotanjung%2Fslff-app-docker%2Fbadge%3Fref%3Dmain&style=flat)](https://actions-badge.atrox.dev//tbriandy/SLFF-JMIOT/goto?ref=main)

**DockerHub Stat(s):**  
![Docker Pulls](https://img.shields.io/docker/pulls//tbriandy/SLFF-JMIOT?logo=docker) ![Docker Image Size (tag)](https://img.shields.io/docker/image-size//tbriandy/SLFF-JMIOT/latest?logo=docker) ![Docker Image Version (latest by date)](https://img.shields.io/docker/v//tbriandy/SLFF-JMIOT?arch=amd64&logo=docker&sort=date)

Dengan Rahmat Tuhan Yang Maha Kuasa. Dengan ini kami membuat aplikasi Control Unit untuk transaksi jalan tol tanpa berhenti menggunakan teknologi RFID. Demikian agar dapat bermanfaat untuk seluruh rakyat Indonesia.

# Persiapan Control Unit
- ## Konfigurasi dan Instalasi Awal
  - Melakukan konfigurasi dan instalasi awal (menggunakan script)
    ```bash
    wget -O - https://raw.githubusercontent.com//tbriandy/SLFF-JMIOT/main/slff_prepare.sh | bash
    ```
  - Mematikan dan menyalakan ulang Control Unit
    ```bash
    sudo reboot
    ```
- ## Parameter
  - Membuat folder **~/slff-data**
      ```bash
      mkdir -p ~/slff-data
      ```
  - Membuat file **slff.yaml** di folder **~/slff-data**
      ```bash
      touch ~/slff-data/slff.yaml
      ```
  - Menulis parameter di file **~/slff-data/slff.yaml**
      ```bash
      nano ~/slff-data/slff.yaml
      ```
      ```yaml
      # -------------------------------------
      # Semanggi 1 Gardu 2
      # -------------------------------------

      # Nama Gerbang
      # ============
      # - Format "{Nama Gerbang} {Nomor Gebang (jika ada)}"
      # - Ditulis menggunakan huruf kapital dan tidak disingkat
      # Contoh:
      # - Semanggi 1
      # - Cikarang Utama 2
      # ============
      nama_gerbang: "Semanggi 1"

      # Tipe Control Unit
      # =================
      # tipe_control_unit: 1 -> Open
      # tipe_control_unit: 2 -> Entrance
      # tipe_control_unit: 3 -> Exit
      # tipe_control_unit: 4 -> Open-Entrance
      # tipe_control_unit: 5 -> Exit-Open
      # =================
      tipe_control_unit: 1

      # Autentikasi
      # ===========
      auth_rfid: true

      # Nomor Gerbang dan Nomor Gardu
      # =============================
      no_gerbang: 10
      no_gardu: 2

      # MID dan TID
      # ===========
      mid: "00000000"
      tid: "00000000"

      # Periph Status Interval
      # ======================
      gto_status_interval: 60
      rfid_status_interval: 60

      # RSS
      # ===
      rss:
          check_url: "0.0.0.0:6000/api/v1/rss/rfid_check"
          store_url: "0.0.0.0:6000/api/v1/rss/store_data"
          jm_code: "00000000000000000000000000000000"

      # Database
      # ========
      # database/host: 'mysql'   -> Nama container jika menggunakan docker
      # database/host: '0.0.0.0' -> IP address jika tidak menggunakan docker
      # ========
      database:
          host: "mysql"
          name: "slff"
          user: "slff"
          password: "slff"

      # Expansion Board
      # ===============
      expansion:
          active: false
          port: "/dev/ttyACM0"
          baud: 1000000

      # RFID
      # ====
      # rfid/type: 0 -> Invengo
      # rfid/type: 1 -> CU1
      # ====
      rfid:
          use_native: true
          port_native: "/dev/ttyS0"
          port: 1
          baud: 115200
          type: 0
          power: 20

      # GTO
      # ===
      gto:
          use_native: true
          port_native: "/dev/ttyS1"
          port: 2
          baud: 19200
      ```
- ## IP Address
  - Masuk ke folder **/etc/netplan/**
    ```bash
    cd /etc/netplan/
    ```
  - Mengubah file yang ada di folder **/etc/netplan/**, misal **00-installer-config.yaml**
    ```bash
    sudo nano 00-installer-config.yaml
    ```
    ```yaml
    network:
      ethernets:
        enp1s0:
          optional: true
          dhcp4: true
        enp2s0:
          optional: true
          dhcp4: false
          addresses: [172.16.212.231/24]
          gateway4: 172.16.212.1
          nameservers:
            addresses: [172.16.212.1,8.8.8.8]
      version: 2
    ```
  - Mengaktifkan konfigurasi IP Address yang baru
    ```bash
    sudo netplan apply
    ```
# Aplikasi Control Unit
- Menjalankan atau memperbarui aplikasi (menggunakan script)
  ```bash
  wget -O - https://raw.githubusercontent.com//tbriandy/SLFF-JMIOT/main/slff_start_or_update.sh | bash
  ```