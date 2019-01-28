#include "SoapyPlutoSDR.hpp"
#include <memory>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <iterator> 
#include <algorithm> 
#include <sys/time.h>

#define BUFFER_SIZE 2048

struct PlutoSDRStream
{
	std::shared_ptr<rx_streamer> rx;
	std::shared_ptr<tx_streamer> tx;
};

std::vector<std::string> SoapyPlutoSDR::getStreamFormats(const int direction, const size_t channel) const
{

	std::vector<std::string> formats;

	formats.push_back(SOAPY_SDR_CS8);
	formats.push_back(SOAPY_SDR_CS16);
	formats.push_back(SOAPY_SDR_CF32);

	return formats;

}

std::string SoapyPlutoSDR::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const
{
	if (direction == SOAPY_SDR_RX) {
		fullScale = 2048; // RX expects 12 bit samples LSB aligned
	}
	else if (direction == SOAPY_SDR_TX) {
		fullScale = 32768; // TX expects 12 bit samples MSB aligned
	}
	return SOAPY_SDR_CS16;
}

SoapySDR::ArgInfoList SoapyPlutoSDR::getStreamArgsInfo(const int direction, const size_t channel) const
{
	SoapySDR::ArgInfoList streamArgs;

	return streamArgs;
}

SoapySDR::Stream *SoapyPlutoSDR::setupStream(
		const int direction,
		const std::string &format,
		const std::vector<size_t> &channels,
		const SoapySDR::Kwargs &args )
{
	std::lock_guard<std::mutex> lock(device_mutex);

	PlutoSDRStream *stream = new PlutoSDRStream();

	//check the format
	plutosdrStreamFormat streamFormat;
	if (format == SOAPY_SDR_CF32) {
		SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32.");
		streamFormat = PLUTO_SDR_CF32;
	}
	else if (format == SOAPY_SDR_CS16) {
		SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16.");
		streamFormat = PLUTO_SDR_CS16;
	}
	else if (format == SOAPY_SDR_CS8) {
		SoapySDR_log(SOAPY_SDR_INFO, "Using format CS8.");
		streamFormat = PLUTO_SDR_CS8;
	}
	else {
		throw std::runtime_error(
			"setupStream invalid format '" + format + "' -- Only CS8, CS16 and CF32 are supported by SoapyPlutoSDR module.");
	}

	if(direction ==SOAPY_SDR_RX){	
		stream->rx = std::shared_ptr<rx_streamer>(new rx_streamer(rx_dev, streamFormat, channels, args));
		rx_stream = stream->rx;
	}

	if (direction == SOAPY_SDR_TX) {

		stream->tx = std::shared_ptr<tx_streamer>(new tx_streamer(tx_dev, streamFormat, channels, args));
	}

	return reinterpret_cast<SoapySDR::Stream *>(stream);

}

void SoapyPlutoSDR::closeStream( SoapySDR::Stream *handle)
{
	std::lock_guard<std::mutex> lock(device_mutex);
	PlutoSDRStream *stream = reinterpret_cast<PlutoSDRStream *>(handle);
	if (stream->rx)
		rx_stream.reset();
	if (stream->tx)
		stream->tx->flush();
	delete stream;

}

size_t SoapyPlutoSDR::getStreamMTU( SoapySDR::Stream *handle) const
{
	return BUFFER_SIZE;
}

int SoapyPlutoSDR::activateStream(
		SoapySDR::Stream *handle,
		const int flags,
		const long long timeNs,
		const size_t numElems )
{
	
	std::lock_guard<std::mutex> lock(device_mutex);
	PlutoSDRStream *stream = reinterpret_cast<PlutoSDRStream *>(handle);

	//if (flags & ~SOAPY_SDR_END_BURST)
		//return SOAPY_SDR_NOT_SUPPORTED;

	if (not stream->rx)
		return 0;
	return stream->rx->start(flags,timeNs,numElems);
}

int SoapyPlutoSDR::deactivateStream(
		SoapySDR::Stream *handle,
		const int flags,
		const long long timeNs )
{
	std::lock_guard<std::mutex> lock(device_mutex);
	PlutoSDRStream *stream = reinterpret_cast<PlutoSDRStream *>(handle);

	if (stream->tx)
		stream->tx->flush();

	if (not stream->rx)
		return 0;

	return stream->rx->stop(flags,timeNs);
}

int SoapyPlutoSDR::readStream(
		SoapySDR::Stream *handle,
		void * const *buffs,
		const size_t numElems,
		int &flags,
		long long &timeNs,
		const long timeoutUs )
{
	PlutoSDRStream *stream = reinterpret_cast<PlutoSDRStream *>(handle);

	return int(stream->rx->recv(buffs, numElems, flags, timeNs, timeoutUs));
}

int SoapyPlutoSDR::writeStream(
		SoapySDR::Stream *handle,
		const void * const *buffs,
		const size_t numElems,
		int &flags,
		const long long timeNs,
		const long timeoutUs )
{
	PlutoSDRStream *stream = reinterpret_cast<PlutoSDRStream *>(handle);

	return stream->tx->send(buffs,numElems,flags,timeNs,timeoutUs);;

}

int SoapyPlutoSDR::readStreamStatus(
		SoapySDR::Stream *stream,
		size_t &chanMask,
		int &flags,
		long long &timeNs,
		const long timeoutUs)
{
	return SOAPY_SDR_NOT_SUPPORTED;
}

void rx_streamer::set_buffer_size_by_samplerate(const size_t _samplerate) {

	uint32_t n = 1, x = uint32_t(_samplerate);
	if ((x >> 16) == 0) { n = n + 16; x = x << 16; }
	if ((x >> 24) == 0) { n = n + 8; x = x << 8; }
	if ((x >> 28) == 0) { n = n + 4; x = x << 4; }
	if ((x >> 30) == 0) { n = n + 2; x = x << 2; }
	n = n - (x >> 31);

	//this->set_buffer_size(std::max(1 << (31 - n - 2), 16384));
	this->set_buffer_size(BUFFER_SIZE);
	SoapySDR_logf(SOAPY_SDR_INFO, "Auto setting Buffer Size: %d", buffer_size);
}

rx_streamer::rx_streamer(const iio_device *_dev, const plutosdrStreamFormat _format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args):
	dev(_dev), format(_format), buffer_size(BUFFER_SIZE), buf(nullptr)

{
	if (dev == nullptr) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "cf-ad9361-lpc not found!");
		throw std::runtime_error("cf-ad9361-lpc not found!");
	}
	unsigned int nb_channels = iio_device_get_channels_count(dev), i;
	for (i = 0; i < nb_channels; i++)
		iio_channel_disable(iio_device_get_channel(dev, i));

	//default to channel 0, if none were specified
	const std::vector<size_t> &channelIDs = channels.empty() ? std::vector<size_t>{0} : channels;

	for (i = 0; i < 1 * 2; i++) {
		struct iio_channel *chn = iio_device_get_channel(dev, i);
		iio_channel_enable(chn);
		channel_list.push_back(chn);
	}
	if ( args.count( "rx_bufflen" ) != 0 ){

		try
		{
			size_t bufferLength = std::stoi(args.at("rx_bufflen"));
			if (bufferLength > 0)
				this->set_buffer_size(bufferLength);
		}
		catch (const std::invalid_argument &){}

	}else{

		long long samplerate;
		
		iio_channel_attr_read_longlong(iio_device_find_channel(dev, "voltage0", false),"sampling_frequency",&samplerate);
		
		this->set_buffer_size_by_samplerate(samplerate);
	
	}

	if (format == PLUTO_SDR_CF32) {
		const float scale = 1.0f / 2048.0f;
		for (int i = 0; i < 4096; ++i)
		{
			lut[i] = (((i + 2048) % 4096) - 2048) * scale;
		}
	}

}

rx_streamer::~rx_streamer() 
{
	if (buf) { iio_buffer_destroy(buf); }

	for(unsigned int i=0;i<channel_list.size(); ++i)
		iio_channel_disable(channel_list[i]);

}



bool do_file_log = false; 
bool do_log = false; 
static bool first_time = true; 
size_t rx_streamer::recv(void * const *buffs,
		const size_t numElems,
		int &flags,
		long long &timeNs,
		const long timeoutUs)
{

	std::unique_lock<std::mutex> lock(mutex);

	if (thread_stopped){

		return SOAPY_SDR_TIMEOUT;
	}
	
	if (do_log) printf("pluto: recv numElems=%d, items_in_buffer=%d, refill=%d\n", numElems, items_in_buffer, please_refill_buffer);

	flags = 0; 
	if (!items_in_buffer) {
		flags = 1; 
		int ret = iio_buffer_refill(buf);
		if (ret < 0) {
		  return 0; 
		}

		items_in_buffer = (unsigned long)ret / iio_buffer_step(buf);
		byte_offset = 0;
	}
	
	if(buffer.size() < numElems*2){
		buffer.reserve(numElems*2);
		buffer.resize(numElems*2);
        }

	size_t items = std::min(items_in_buffer,numElems);

	int16_t dst = 0;
	uintptr_t src_ptr, dst_ptr = (uintptr_t)&dst;
	ptrdiff_t buf_step = iio_buffer_step(buf);

	if (direct_copy) {
  	    if (do_log) {
		struct timeval t;
		gettimeofday(&t, NULL);        	
 		printf("pluto: copying %d items, t=%.1f\n", items, (float) t.tv_usec/1000);
	    }
		// optimize for single RX, 2 channel (I/Q), same endianess direct copy
		src_ptr = (uintptr_t)iio_buffer_start(buf) + byte_offset;
		memcpy((void *)buffer.data(),(void *) src_ptr, 2 * sizeof(int16_t) * items);
		float *samples_cf32 = (float *)buffs[0];
		for (size_t j = 0; j < items*2; ++j) {
	          samples_cf32[j] = lut[buffer[j] & 0x0FFF];
	        }		
	}
	else{

		for (unsigned int i = 0; i < channel_list.size(); i++) {
			iio_channel *chn = channel_list[i];
			unsigned int index = i / 2;

			src_ptr = (uintptr_t)iio_buffer_first(buf, chn) + byte_offset;

			if (format == PLUTO_SDR_CS16) {
				int16_t *samples_cs16 = (int16_t *)buffs[index];
				for (size_t j = 0; j < items; ++j) {
					iio_channel_convert(chn, (void *)dst_ptr, (const void *)src_ptr);
					src_ptr += buf_step;
					samples_cs16[j * 2 + i] = dst;
				}		
			}else if (format == PLUTO_SDR_CF32) {
				float *samples_cf32 = (float *)buffs[index];
				for (size_t j = 0; j < items; ++j) {
					iio_channel_convert(chn, (void *)dst_ptr, (const void *)src_ptr);
					src_ptr += buf_step;
					samples_cf32[j * 2 + i] = lut[dst & 0x0FFF];
				}
			}
			else if (format == PLUTO_SDR_CS8) {
				int8_t *samples_cs8 = (int8_t *)buffs[index];
				for (size_t j = 0; j < items; ++j) {
					iio_channel_convert(chn, (void *)dst_ptr, (const void *)src_ptr);
					src_ptr += buf_step;
					samples_cs8[j * 2 + i] = dst >> 4;
				}
			}

		}
        }

	items_in_buffer -= items;
	byte_offset += items * iio_buffer_step(buf);

	uint32_t val;
	if (iio_device_reg_read((struct iio_device*) dev, 0x80000088, &val)) {
//		printf("Failed to load status register\n");
	}
	if (val & 1)
	  fprintf(stderr, "[IIO] Underflow detected in RX\n");
	if (val & 4)
	  fprintf(stderr, "[IIO] Overflow detected in RX\n");

	if (do_log) printf("pluto: items_in_buffer is now %d\n", items_in_buffer);
	return(items);

}

int rx_streamer::start(const int flags,
		const long long timeNs,
		const size_t numElems)
{
	std::unique_lock<std::mutex> lock(mutex);

	items_in_buffer = 0;
	please_refill_buffer = false;
	thread_stopped = false;

	printf("RX buffer size: %d samples\n", buffer_size);
	buf = iio_device_create_buffer(dev, buffer_size, false);

	if (buf) {
	/*
		refill_thd = std::thread(&rx_streamer::refill_thread, this);
		struct sched_param p; 
		p.sched_priority = sched_get_priority_max(SCHED_FIFO); 
		if (pthread_setschedparam(refill_thd.native_handle(), SCHED_FIFO, &p)) {
			printf("Error setting priority\n");
		}
	*/	
	} else {
		SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to create buffer!");
		throw std::runtime_error("Unable to create buffer!\n");
	}

	direct_copy = has_direct_copy();
	direct_copy = true;

	return 0;

}

int rx_streamer::stop(const int flags,
		const long long timeNs)
{
	if (buf)
		iio_buffer_cancel(buf);

	std::unique_lock<std::mutex> lock(mutex);

	please_refill_buffer = true;
	cond.notify_all();
	lock.unlock();

	refill_thd.join();

	if (buf) {
		iio_buffer_destroy(buf);
		buf = nullptr;
	}

	return 0;


}

void rx_streamer::set_buffer_size(const size_t _buffer_size){

	std::unique_lock<std::mutex> lock(mutex);

	if (buf && this->buffer_size != _buffer_size) {
		iio_buffer_destroy(buf);
		printf("pluto: resizing buffer according to sample rate\n");
		buf = iio_device_create_buffer(dev, _buffer_size, false);
		if (!buf) {
			SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to create buffer!");
			throw std::runtime_error("Unable to create buffer!\n");
		}
			
	}

	this->buffer_size=_buffer_size;

}
void rx_streamer::reset_buffer_size(const size_t _buffer_size) {
       new_buffer_size = _buffer_size;
       resize_buffer = true;
}

void rx_streamer::refill_thread(){

	std::unique_lock<std::mutex> lock(mutex);
	ssize_t ret;

	for (;;) {

		while (!please_refill_buffer)
			cond.wait(lock);

		please_refill_buffer = false;

		lock.unlock();
		ret = iio_buffer_refill(buf);
		lock.lock();
		if (ret < 0)
			break;

		items_in_buffer = (unsigned long)ret / iio_buffer_step(buf);
		byte_offset = 0;
		cond2.notify_one();

	}
	thread_stopped = true;
	cond2.notify_all();
}

// return wether can we optimize for single RX, 2 channel (I/Q), same endianess direct copy
bool rx_streamer::has_direct_copy()
{
	if (channel_list.size() != 2) // one RX with I/Q
		return false;

	ptrdiff_t buf_step = iio_buffer_step(buf);

	if (buf_step != 2 * sizeof(int16_t)) {
		return false;
        }

	if (iio_buffer_start(buf) != iio_buffer_first(buf, channel_list[0]))
		return false;


	int16_t test_dst, test_src = 0x1234;
	iio_channel_convert(channel_list[0], (void *)&test_dst, (const void *)&test_src);

	return test_src == test_dst;

}


tx_streamer::tx_streamer(const iio_device *_dev, const plutosdrStreamFormat _format, const std::vector<size_t> &channels, const SoapySDR::Kwargs &args) :
	dev(_dev), format(_format), buf(nullptr)
{

	if (dev == nullptr) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "cf-ad9361-dds-core-lpc not found!");
		throw std::runtime_error("cf-ad9361-dds-core-lpc not found!");
	}

	unsigned int nb_channels = iio_device_get_channels_count(dev), i;
	for (i = 0; i < nb_channels; i++)
		iio_channel_disable(iio_device_get_channel(dev, i));

	//default to channel 0, if none were specified
	const std::vector<size_t> &channelIDs = channels.empty() ? std::vector<size_t>{0} : channels;

	for (i = 0; i < 1 * 2; i++) {
		iio_channel *chn = iio_device_get_channel(dev, i);
		iio_channel_enable(chn);
		channel_list.push_back(chn);
	}
	buf_size = BUFFER_SIZE;
	if ( args.count( "tx_bufflen" ) != 0 ){

		try
		{
			size_t bufferLength = std::stoi(args.at("tx_bufflen"));
			if (bufferLength > 0)
				buf_size = BUFFER_SIZE;
		}
		catch (const std::invalid_argument &){}

	}

	printf("TX buffer size: %d samples\n", buf_size);
	items_in_buf = 0;
	buf = iio_device_create_buffer(dev, buf_size, false);
	if (!buf) {
		SoapySDR_logf(SOAPY_SDR_ERROR, "Unable to create buffer!");
		throw std::runtime_error("Unable to create buffer!");
	}

	direct_copy = has_direct_copy();

}

tx_streamer::~tx_streamer(){

	if (buf) { iio_buffer_destroy(buf); }

	for(unsigned int i=0;i<channel_list.size(); ++i)
		iio_channel_disable(channel_list[i]);

}

int tx_streamer::send(	const void * const *buffs,
		const size_t numElems,
		int &flags,
		const long long timeNs,
		const long timeoutUs )

{
	std::lock_guard<std::mutex> lock(mutex);
	size_t items = std::min(buf_size - items_in_buf, numElems);

	int16_t src = 0;
	uintptr_t dst_ptr, src_ptr = (uintptr_t)&src;
	ptrdiff_t buf_step = iio_buffer_step(buf);
	if(buffer.size() < numElems*2){
		buffer.reserve(numElems*2);
		buffer.resize(numElems*2);
        }
        
        
//direct_copy = false;
	if (direct_copy) {
	        // optimize for single TX, 2 channel (I/Q), same endianess direct copy
                float *samples_cf32 = (float *)buffs[0];
 	        for (size_t j = 0; j < 2*numElems; ++j) {
 	                buffer[j]=(int16_t)(samples_cf32[j]*32767.999f);
 	        }

	        dst_ptr = (uintptr_t)iio_buffer_start(buf) + items_in_buf * 2 * sizeof(int16_t);
		memcpy((void *)dst_ptr, buffer.data(), 2 * sizeof(int16_t) * items);
	}
	else {

	for (unsigned int i = 0; i < channel_list.size(); i++) {
		iio_channel *chn = channel_list[i];
		unsigned int index = i / 2;

		dst_ptr = (uintptr_t)iio_buffer_first(buf, chn) + items_in_buf * buf_step;

		// note that TX expects samples MSB aligned, unlike RX which is LSB aligned
		if (format == PLUTO_SDR_CS16) {

			int16_t *samples_cs16 = (int16_t *)buffs[index];
			for (size_t j = 0; j < items; ++j) {
				src = samples_cs16[j*2+i];
				iio_channel_convert_inverse(chn, (void *)dst_ptr, (const void *)src_ptr);
				dst_ptr += buf_step;
			}
		}
		else if (format == PLUTO_SDR_CF32) {

			float *samples_cf32 = (float *)buffs[index];
			for (size_t j = 0; j < items; ++j) {
				src = (int16_t)(samples_cf32[j*2+i] * 32767.999f); // 32767.999f (0x46ffffff) will ensure better distribution
				iio_channel_convert_inverse(chn, (void *)dst_ptr, (const void *)src_ptr);
				dst_ptr += buf_step;
			}
		}
		else if (format == PLUTO_SDR_CS8) {

			int8_t *samples_cs8 = (int8_t *)buffs[index];
			for (size_t j = 0; j < items; ++j) {
				src = (int16_t)(samples_cs8[j*2+i] << 8);
				iio_channel_convert_inverse(chn, (void *)dst_ptr, (const void *)src_ptr);
				dst_ptr += buf_step;
			}
		}
	}
     }

	items_in_buf += items;

	if (items_in_buf == buf_size || (flags & SOAPY_SDR_END_BURST && numElems == items)) {
		int ret = send_buf();
		if (ret < 0) {
			return SOAPY_SDR_ERROR;
		}

		if ((size_t)ret != buf_size) {
			return SOAPY_SDR_ERROR;
		}
	}

	uint32_t val;
	if (iio_device_reg_read((struct iio_device*) dev, 0x80000088, &val)) {
//		printf("Failed to load status register\n");
	}
	if (val & 1)
	  fprintf(stderr, "[IIO] Underflow detected in TX\n");
	if (val & 4)
	  fprintf(stderr, "[IIO] Overflow detected in TX\n");

	return items;

}

int tx_streamer::flush()
{
	std::lock_guard<std::mutex> lock(mutex);

	return send_buf();

}

int tx_streamer::send_buf()
{

	if (items_in_buf > 0) {
		if (items_in_buf < buf_size) {
			ptrdiff_t buf_step = iio_buffer_step(buf);
			uintptr_t buf_ptr = (uintptr_t)iio_buffer_start(buf) + items_in_buf * buf_step;
			uintptr_t buf_end = (uintptr_t)iio_buffer_end(buf);

			memset((void *)buf_ptr, 0, buf_end - buf_ptr);
		}

		struct timeval t; 
		gettimeofday(&t, NULL);
		ssize_t ret = iio_buffer_push(buf);
		items_in_buf = 0;

		if (ret < 0) {
			return ret;
		}

		return int(ret / iio_buffer_step(buf));
	}

	return 0;

}

// return wether can we optimize for single TX, 2 channel (I/Q), same endianess direct copy
bool tx_streamer::has_direct_copy()
{

	if (channel_list.size() != 2) // one TX with I/Q
		return false;

	ptrdiff_t buf_step = iio_buffer_step(buf);

	if (buf_step != 2 * sizeof(int16_t))
		return false;

	if (iio_buffer_start(buf) != iio_buffer_first(buf, channel_list[0]))
		return false;

	int16_t test_dst, test_src = 0x1234;
	iio_channel_convert_inverse(channel_list[0], (void *)&test_dst, (const void *)&test_src);

	return test_src == test_dst;

}
