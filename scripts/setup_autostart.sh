#!/bin/bash
# LODOS Albatros ROS2 Otomatik Başlatma (systemd) Kurulum Betiği

# Sudo yetkisi kontrolü
if [ "$EUID" -ne 0 ]; then
  echo "HATA: Lütfen bu betiği sudo yetkisiyle çalıştırın."
  echo "Kullanım: sudo bash setup_autostart.sh"
  exit 1
fi

SERVICE_FILE="/etc/systemd/system/albatros.service"
WORKSPACE_DIR="/home/mhmd/Downloads/Lodos"

echo "⚙️  albatros.service dosyası oluşturuluyor..."

# Service dosyasının içeriğini /etc/systemd/system/ içine yazıyoruz
cat <<EOF > $SERVICE_FILE
[Unit]
Description=LODOS Albatros ROS2 Otonomi Başlatıcı
After=network.target

[Service]
Type=simple
User=mhmd
WorkingDirectory=$WORKSPACE_DIR
# Önce JAZZY'i, sonra kendi workspace'imizi source edip launch dosyasını tetikliyoruz
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && source $WORKSPACE_DIR/install/setup.bash && ros2 launch albatros_tahta albatros.launch.py"
# Node'lardan biri çökerse 5 saniye sonra tekrar başlat
Restart=on-failure
RestartSec=5
# Print komutlarının loglara anında düşmesi için
Environment="PYTHONUNBUFFERED=1"

[Install]
WantedBy=multi-user.target
EOF

# Systemd'yi yenile ve servisi başlangıçta çalışacak şekilde aktif et
echo "🔄 systemd arka plan programları yenileniyor..."
systemctl daemon-reload

echo "✅ Servis başlangıçta (boot) çalışmak üzere aktifleştiriliyor..."
systemctl enable albatros.service

echo "================================================================"
echo "🎉 KURULUM TAMAMLANDI!"
echo "Artık Raspberry Pi her açıldığında ROS2 launch otomatik çalışacak."
echo "================================================================"
echo " "
echo "Faydalı Komutlar (Terminalden yönetmek istersen):"
echo "- Servisi HEMEN başlatmak için  : sudo systemctl start albatros"
echo "- Servisi DURDURMAK için        : sudo systemctl stop albatros"
echo "- Servisin DURUMUNU görmek için : sudo systemctl status albatros"
echo "- Canlı LOGLARI izlemek için    : journalctl -u albatros -f"
