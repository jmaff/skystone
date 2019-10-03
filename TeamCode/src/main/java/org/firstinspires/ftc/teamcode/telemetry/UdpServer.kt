package org.firstinspires.ftc.teamcode.telemetry

import android.os.SystemClock
import com.qualcomm.robotcore.util.Range
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.util.concurrent.Semaphore

class UdpServer(val clientPort: Int) : Runnable {
    var active = true
    val sendLock = Semaphore(1)
    var lastSendMillis = 0L
    var currentUpdate = ""
    var lastUpdate = ""

    override fun run() {
        while (true) {
            if (!active) break
            try {
                if (System.currentTimeMillis() - lastSendMillis < 50) {
                    continue
                }
                lastSendMillis = System.currentTimeMillis()
                sendLock.acquire()
                if (currentUpdate.isNotEmpty()) {
                    splitAndSend(currentUpdate)
                    currentUpdate = ""
                } else {
                    if (lastUpdate.isNotEmpty()) {
                        splitAndSend(lastUpdate)
                        lastUpdate = ""
                    }
                }
                sendLock.release()
            } catch (e: InterruptedException) {
                e.printStackTrace()
            }
        }
    }

    fun splitAndSend(message: String) {
        var startIndex = 0
        var endIndex: Int

        do {
            endIndex = Range.clip(startIndex + 600, 0, message.length - 1)
            while (message[endIndex] != '%') {
                endIndex--
            }

            sendUdpRaw(message.substring(startIndex, endIndex + 1))
            startIndex = endIndex + 1
        } while (endIndex != message.length - 1)
    }

    private fun sendUdpRaw(message: String) {
        val serverSocket = DatagramSocket().use {
            val datagramPacket = DatagramPacket(
                    message.toByteArray(),
                    message.length,
                    InetAddress.getByName("192.168.49.11"), // TODO: Change address to debug laptop
                    clientPort
            )

            it.send(datagramPacket)
        }
    }

    fun addMessage(message: String) {
        if (!sendLock.tryAcquire()) {
            lastUpdate = message
        } else {
            currentUpdate = message
            sendLock.release()
        }
    }
}