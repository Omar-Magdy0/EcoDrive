#include "Usbxch.h"

UsbxchLibusb::~UsbxchLibusb() { disconnect(); }

bool UsbxchLibusb::connect(const std::string &serial_number,
                           uint16_t vid,
                           uint16_t pid,
                           int interface_number,
                           unsigned char bulk_in_ep,
                           unsigned char bulk_out_ep,
                           int timeout_ms)
{
    disconnect();

    if (libusb_init(&context_) != LIBUSB_SUCCESS)
        return false;

    libusb_device **dev_list = nullptr;
    ssize_t dev_count = libusb_get_device_list(context_, &dev_list);
    if (dev_count < 0)
    {
        libusb_exit(context_);
        context_ = nullptr;
        return false;
    }

    bool found = false;

    for (ssize_t i = 0; i < dev_count && !found; ++i)
    {
        libusb_device *dev = dev_list[i];

        libusb_device_descriptor desc;
        if (libusb_get_device_descriptor(dev, &desc) != LIBUSB_SUCCESS)
            continue;

        if (desc.idVendor != vid || desc.idProduct != pid)
            continue;

        if (!serial_number.empty())
        {
            if (desc.iSerialNumber == 0)
                continue;

            libusb_device_handle *tmp = nullptr;
            if (libusb_open(dev, &tmp) != LIBUSB_SUCCESS)
                continue;

            char serial[256] = {};
            int rc = libusb_get_string_descriptor_ascii(
                tmp,
                desc.iSerialNumber,
                reinterpret_cast<unsigned char *>(serial),
                sizeof(serial));

            libusb_close(tmp);

            if (rc <= 0 || serial != serial_number)
                continue;
        }

        if (libusb_open(dev, &handle_) != LIBUSB_SUCCESS)
            continue;

        libusb_set_auto_detach_kernel_driver(handle_, 1);

        int current_config = 0;
        libusb_get_configuration(handle_, &current_config);

        libusb_config_descriptor *cfg = nullptr;
        int rc = libusb_get_active_config_descriptor(dev, &cfg);

        if (rc != LIBUSB_SUCCESS)
        {
            libusb_close(handle_);
            handle_ = nullptr;
            continue;
        }

        rc = libusb_claim_interface(handle_, interface_number);

        libusb_free_config_descriptor(cfg);

        if (rc != LIBUSB_SUCCESS)
        {
            libusb_close(handle_);
            handle_ = nullptr;
            continue;
        }

        found = true;
    }

    libusb_free_device_list(dev_list, 1);

    if (!found)
    {
        libusb_exit(context_);
        context_ = nullptr;
        return false;
    }

    interface_number_ = interface_number;
    bulk_in_ep_ = bulk_in_ep;
    bulk_out_ep_ = bulk_out_ep;
    timeout_ms_ = timeout_ms;

    connected_ = true;
    running_ = true;

    io_thread_ = std::thread([this]
    {
        ioLoop();
    });

    return true;
}

void UsbxchLibusb::disconnect()
{
    if (!connected_ && !handle_)
        return;

    connected_ = false;
    running_ = false;
    tx_cv_.notify_one();

    if (io_thread_.joinable())
        io_thread_.join();

    if (handle_)
    {
        libusb_release_interface(handle_, interface_number_);
        libusb_close(handle_);
        handle_ = nullptr;
    }

    if (context_)
    {
        libusb_exit(context_);
        context_ = nullptr;
    }
}

bool UsbxchLibusb::isConnected() const { return connected_; }

int UsbxchLibusb::write(const uint8_t *data, size_t length)
{
    if (!connected_ || !handle_ || !data || length == 0)
        return false;

    std::lock_guard<std::mutex> lock(tx_mutex_);

    int transferred = 0;

    int rc = libusb_bulk_transfer(
        handle_,
        bulk_out_ep_,
        const_cast<unsigned char *>(data),
        static_cast<int>(length),
        &transferred,
        timeout_ms_);

    if (rc != LIBUSB_SUCCESS)
    {
        std::cerr << "[USBXCH] TX error: "
                  << libusb_error_name(rc)
                  << '\n';

        return false;
    }

    speed_tracker_.update(0, transferred);
    return transferred == static_cast<int>(length);
}

int UsbxchLibusb::read(uint8_t *data, size_t max_len)
{
    if (!data || max_len == 0)
        return 0;
    return rx_buffer_.pop(data, max_len);
}

size_t UsbxchLibusb::available()
{
    return rx_buffer_.available();
}

double UsbxchLibusb::getRxSpeedBytesPerSecond() const { return speed_tracker_.rxBps(); }
double UsbxchLibusb::getTxSpeedBytesPerSecond() const { return speed_tracker_.txBps(); }
void UsbxchLibusb::resetSpeeds() { speed_tracker_.reset(); }

void UsbxchLibusb::ioLoop()
{
    std::array<uint8_t, 4096> transfer;

    while (running_)
    {
        int transferred = 0;

        int rc = libusb_bulk_transfer(
            handle_,
            bulk_in_ep_,
            transfer.data(),
            transfer.size(),
            &transferred,
            1);

        if (rc == LIBUSB_SUCCESS)
        {
            if (transferred)
            {
                size_t pushed =
                    rx_buffer_.push(
                        transfer.data(),
                        transferred);

                speed_tracker_.update(transferred, 0);

                if (pushed != static_cast<size_t>(transferred))
                {
                    std::cerr
                        << "[USBXCH] RX overflow\n";
                }
            }
        }
        else if (rc != LIBUSB_ERROR_TIMEOUT)
        {
            std::cerr
                << "[USBXCH] RX error: "
                << libusb_error_name(rc)
                << '\n';

            connected_ = false;
            running_ = false;
        }
    }
}

UsbxchTcp::~UsbxchTcp()
{
    disconnect();
}

#include <iostream>
bool UsbxchTcp::connect(const std::string &serial_number, uint16_t vid, uint16_t pid, int interface_number, unsigned char bulk_in_ep, unsigned char bulk_out_ep, int timeout_ms)
{
    disconnect();

    if (!client.connect("127.0.0.1", 4001))
    {
        return false;
    }

    connected_ = true;
    running_ = true;
    timeout_ms_ = timeout_ms;
    interface_number_ = interface_number;

    io_thread_ = std::thread([this]()
                             { ioLoop(); });
    std::cout << "USBXCH CONNECTED TO TCP localhost:4001" << std::endl;
    return true;
}

void UsbxchTcp::disconnect()
{
    connected_ = false;
    running_ = false;

    if (io_thread_.joinable())
        io_thread_.join();

    client.disconnect();
}

bool UsbxchTcp::isConnected() const
{
    return client.is_connected();
}

int UsbxchTcp::write(const uint8_t *data, size_t length)
{
    const int result = client.write(data, length);
    if (result > 0)
    {
        speed_tracker_.update(0, static_cast<size_t>(result));
    }
    return result;
}

int UsbxchTcp::read(uint8_t *data, size_t max_len)
{
    const int result = client.read(data, max_len);
    if (result > 0)
    {
        speed_tracker_.update(static_cast<size_t>(result), 0);
    }
    return result;
}

size_t UsbxchTcp::available()
{
    return client.available();
}

void UsbxchTcp::ioLoop()
{
    while (running_)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

double UsbxchTcp::getRxSpeedBytesPerSecond() const { return speed_tracker_.rxBps(); }
double UsbxchTcp::getTxSpeedBytesPerSecond() const { return speed_tracker_.txBps(); }
void UsbxchTcp::resetSpeeds() { speed_tracker_.reset(); }
