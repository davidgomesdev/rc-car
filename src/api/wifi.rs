//! API: Wi-Fi service. Brings up the station + access-point netifs, tries to
//! join the configured network, and falls back to access-point mode. Owns the
//! Wi-Fi driver for the program's lifetime.

use anyhow::Result;
use std::net::Ipv4Addr;

use embedded_svc::ipv4::{Mask, RouterConfiguration, Subnet};
use esp_idf_hal::modem::Modem;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::ipv4;
use esp_idf_svc::ipv4::Configuration;
use esp_idf_svc::netif::{EspNetif, NetifConfiguration};
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{
    AccessPointConfiguration, AuthMethod, BlockingWifi, ClientConfiguration,
    Configuration as WifiConfig, EspWifi, WifiDriver,
};

/// Owns the Wi-Fi driver and stays alive as long as the firmware runs.
pub struct WifiService {
    wifi: BlockingWifi<EspWifi<'static>>,
}

impl WifiService {
    /// Bring up Wi-Fi: try the configured client network, falling back to AP.
    pub fn connect(
        modem: Modem<'static>,
        sys_loop: EspSystemEventLoop,
        nvs: EspDefaultNvsPartition,
    ) -> Result<Self> {
        let sta_netif = EspNetif::new_with_conf(&NetifConfiguration {
            ip_configuration: Some(ipv4::Configuration::Client(
                ipv4::ClientConfiguration::DHCP(ipv4::DHCPClientSettings {
                    hostname: Some("rc-car".try_into().unwrap()),
                }),
            )),
            ..NetifConfiguration::wifi_default_client()
        })?;
        let ap_netif = EspNetif::new_with_conf(&NetifConfiguration {
            ip_configuration: Some(Configuration::Router(RouterConfiguration {
                subnet: Subnet {
                    gateway: Ipv4Addr::from_octets([192u8, 168u8, 1u8, 1u8]),
                    mask: Mask(24),
                },
                dhcp_enabled: true,
                dns: None,
                secondary_dns: None,
            })),
            ..NetifConfiguration::wifi_default_router()
        })?;

        let mut wifi = BlockingWifi::wrap(
            EspWifi::wrap_all(
                WifiDriver::new(modem, sys_loop.clone(), Some(nvs))?,
                sta_netif,
                #[cfg(esp_idf_esp_wifi_softap_support)]
                ap_netif,
            )?,
            sys_loop,
        )?;

        if let Err(e) = Self::connect_to_client(&mut wifi) {
            log::error!("Failed to connect to client WiFi: {e}");
            Self::start_access_point(&mut wifi)?;
        }

        Ok(Self { wifi })
    }

    /// Resolve the assigned IP: station address if up, otherwise the AP address.
    pub fn ip(&self) -> Result<Ipv4Addr> {
        let ip = if self.wifi.wifi().sta_netif().is_up()? {
            self.wifi.wifi().sta_netif().get_ip_info()?.ip
        } else {
            self.wifi.wifi().ap_netif().get_ip_info()?.ip
        };
        Ok(ip)
    }

    fn connect_to_client(wifi: &mut BlockingWifi<EspWifi<'static>>) -> Result<()> {
        let ssid = env!("CLIENT_WIFI_SSID");
        let password = env!("CLIENT_WIFI_PASSWORD");

        log::info!("Attempting to connect to WiFi SSID: {}", ssid);
        wifi.set_configuration(&WifiConfig::Client(ClientConfiguration {
            ssid: ssid.try_into().unwrap(),
            password: password.try_into().unwrap(),
            auth_method: AuthMethod::WPA2Personal,
            ..Default::default()
        }))?;
        wifi.start()?;
        wifi.connect()?;
        Ok(())
    }

    fn start_access_point(wifi: &mut BlockingWifi<EspWifi<'static>>) -> Result<()> {
        let ssid = env!("AP_WIFI_SSID");
        let password = env!("AP_WIFI_PASSWORD");

        log::info!("Switching to access point mode");
        wifi.stop()?;
        wifi.set_configuration(&WifiConfig::AccessPoint(AccessPointConfiguration {
            ssid: ssid.try_into()?,
            auth_method: AuthMethod::WPA2Personal,
            password: password.try_into()?,
            channel: 1,
            ..Default::default()
        }))?;
        wifi.start()?;
        wifi.wait_netif_up()?;

        let ip = wifi.wifi().ap_netif().get_ip_info()?.ip;
        log::info!("Wi-Fi AP up — SSID: {ssid}  IP: {ip}");
        Ok(())
    }
}
