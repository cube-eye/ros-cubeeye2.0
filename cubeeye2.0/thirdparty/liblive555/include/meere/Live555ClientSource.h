/*
 * Live555ClientSource.h
 *
 *  Created on: 2022. 10. 4.
 *      Author: erato
 */

/**
 Copyright (c) 2022 erato <yjbae@meerecompany.com>

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as
 published by the Free Software Foundation, either version 3 of the
 License, or (at your option) any later version.
  
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU Lesser General Public License for more details.
  
 You should have received a copy of the GNU Lesser General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIVE555CLIENTSOURCE_H_
#define LIVE555CLIENTSOURCE_H_

#include <memory>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <future>
#include "liveMedia.hh"

#ifdef _WIN32
#ifdef _BUILD_FOR_DLL_EXPORT
#define _DECL_DLL	__declspec(dllexport)
#else //_BUILD_FOR_DLL_EXPORT
#define _DECL_DLL	__declspec(dllimport)
#endif //!_BUILD_FOR_DLL_EXPORT
#define _DECL_CALL	__stdcall
#else //!_WIN32
#define _DECL_DLL
#define _DECL_CALL
#endif //_WIN32


class _DECL_DLL Live555ClientSource
{
public:
	class _DECL_DLL Listener
	{
	public:
		enum class LogLevel {
			Verbose,
			Info,
			Debug,
			Warn,
			Error
		};

	public:
		virtual void _DECL_CALL onReceivedFrameDataFromLive555(
				const uint8_t* data, size_t size, uint16_t width, uint16_t height,
				uint64_t timestamp, const std::string& codec = "", const std::string& path = "") = 0;
		virtual void _DECL_CALL onReceivedLogFromLive555(LogLevel level, const std::string& log) = 0;
	protected:
		Listener() = default;
		virtual ~Listener() = default;
	};

public:
	explicit Live555ClientSource(Listener* listener);
	Live555ClientSource(const Live555ClientSource&) = delete;
	Live555ClientSource& operator=(const Live555ClientSource&) = delete;
	Live555ClientSource(Live555ClientSource&&) = delete;
	Live555ClientSource& operator=(Live555ClientSource&&) = delete;
	virtual ~Live555ClientSource();

public:
	virtual bool _DECL_CALL open();
	virtual bool _DECL_CALL run(const std::string& url);
	virtual void _DECL_CALL stop();
	virtual void _DECL_CALL close();

protected:
	class RTSPClientSink : public MediaSink
	{
	public:
		explicit RTSPClientSink(UsageEnvironment& env, MediaSubsession& subsession, Live555ClientSource* source);
		RTSPClientSink(const RTSPClientSink&) = delete;
		RTSPClientSink(RTSPClientSink&&) = delete;
		RTSPClientSink& operator=(const RTSPClientSink&) = delete;
		RTSPClientSink& operator=(RTSPClientSink&&) = delete;
		virtual ~RTSPClientSink();

	protected:
		virtual Boolean continuePlaying();

	protected:
		static void afterGettingFrame(void* clientData, unsigned frameSize, \
				unsigned numTruncatedBytes, struct timeval presentationTime, unsigned durationInMicroseconds);

	private:
		MediaSubsession& mMediaSubsession;
		std::unique_ptr<uint8_t[]> mUptrReceivedBuffer;
		Live555ClientSource* mClientSource;
	};

	class RTSPClientImpl : public RTSPClient
	{
	public:
		explicit RTSPClientImpl(UsageEnvironment& env, char const* rtspURL, Live555ClientSource* source);
		RTSPClientImpl(const RTSPClientImpl&) = delete;
		RTSPClientImpl(RTSPClientImpl&&) = delete;
		RTSPClientImpl& operator=(const RTSPClientImpl&) = delete;
		RTSPClientImpl& operator=(RTSPClientImpl&&) = delete;
		virtual ~RTSPClientImpl() = default;

	public:
		struct StreamClientState {
			StreamClientState();
			virtual ~StreamClientState();

			MediaSubsessionIterator* iter;
			MediaSession* session;
			MediaSubsession* subsession;
			TaskToken streamTimerTask;
			double duration;
		} mStreamClientState;
		Live555ClientSource* mClientSource;
	};

protected:
	static Listener* listener(RTSPClient* client);
	static void continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode, char* resultString);
	static void	continueAfterSETUP(RTSPClient* rtspClient, int resultCode, char* resultString);
	static void	continueAfterPLAY(RTSPClient* rtspClient, int resultCode, char* resultString);
	static void	subsessionAfterPlaying(void* clientData); // called when a stream's subsession (e.g., audio or video substream) ends
	static void	subsessionByeHandler(void* clientData, char const* reason);
	static void	streamTimerHandler(void* clientData);
	static void	setupNextSubsession(RTSPClient* rtspClient);
	static void	shutdownStream(RTSPClient* rtspClient);

protected:
	std::unique_ptr<TaskScheduler> mUptrLive555Scheduler;
	std::unique_ptr<UsageEnvironment, std::function<void(UsageEnvironment*)>> mUptrLive555Environment;
	std::unique_ptr<RTSPClientImpl, std::function<void(RTSPClientImpl*)>> mUptrLive555Client;
	char mLive555EventLoopWatchVariable;
	Listener* mListener;
	std::mutex mSinkLock;
	std::future<void> mStopScheduler;
};


using live555_client_log_level = Live555ClientSource::Listener::LogLevel;
using live555_client_source_listener = Live555ClientSource::Listener;
using sptr_live555_client_source = std::shared_ptr<Live555ClientSource>;

_DECL_DLL sptr_live555_client_source _DECL_CALL create_live555_client_source(live555_client_source_listener* listener);

#endif /* LIVE555CLIENTSOURCE_H_ */
