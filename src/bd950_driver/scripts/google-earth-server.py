#!/usr/bin/env python

import BaseHTTPServer
import rospy
from sensor_msgs.msg import NavSatFix
from Queue import Queue
import simplekml
import threading

coordinate_queue = Queue()

def rx_fix_msg(data):
    # NOTE: its lon, and then lat!
    coordinate_queue.put([(data.longitude, data.latitude)])

class MyHandler(BaseHTTPServer.BaseHTTPRequestHandler):
    current_coordinates = [(0, 0)]
    def do_GET(s):
        s.send_response(200)
        s.send_header("Content-type", "text/xml")
        s.end_headers()

        if s.path == "/rover":
            while coordinate_queue.empty() is False:
                current_coordinates = coordinate_queue.get_nowait()

            kml = simplekml.Kml()
            kml.newpoint(name="Rover", coords=current_coordinates)
            s.wfile.write(kml.kml(format=True))

        else:
            s.wfile.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>")
            s.wfile.write("<kml xmlns=\"http://earth.google.com/kml/2.1\">")
            s.wfile.write("<Document>")
            s.wfile.write("  <NetworkLink>")
            s.wfile.write("    <Link>")
            s.wfile.write("      <href>http://{0}/rover</href>".format(s.headers["Host"]))
            s.wfile.write("      <refreshMode>onInterval</refreshMode>")
            s.wfile.write("    <refreshInterval>1</refreshInterval>")
            s.wfile.write("    </Link>")
            s.wfile.write("  </NetworkLink>")
            s.wfile.write("</Document>")
            s.wfile.write("</kml>")


rospy.init_node("google_earth_server")
rospy.Subscriber("/bd950_driver_node/fix", NavSatFix, rx_fix_msg, queue_size=10)

server_class = BaseHTTPServer.HTTPServer
httpd = server_class(("0.0.0.0", 8080), MyHandler)
server_thread = threading.Thread(target=httpd.serve_forever)
server_thread.daemon = True
server_thread.start()

rospy.spin()
httpd.server_close()

