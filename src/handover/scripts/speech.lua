#!/usr/bin/lua

-- Copyright: (C) 2021 iCub Facility - Istituto Italiano di Tecnologia (IIT)

-- Authors: Vadim Tikhanoff <vadim.tikhanoff@iit.it>
--          Elisa Maiettini <elisa.maiettini@iit.it>
--          Nicola Piga <nicola.piga@iit.it>

local signal = require("posix.signal")
require("yarp")

rf = yarp.ResourceFinder()
rf:setVerbose(false)
rf:configure(arg)

---------------------------------------
-- setting up ctrl-c signal handling --
---------------------------------------

interrupting = false
signal.signal(signal.SIGINT, function(signum)
    interrupting = true
end)

signal.signal(signal.SIGTERM, function(signum)
    interrupting = true
end)

---------------------------------------
-- yarp port initializations         --
---------------------------------------
yarp.Network()

port_speech_recog = yarp.Port()
port_speech_recog:open("/roft-track-n-grasp/speech/speech_recognizer:o")

port_output = yarp.Port()
port_output:open("/roft-track-n-grasp/speech/object:o")

ret = true
ret = ret and yarp.NetworkBase_connect(port_speech_recog:getName(), "/speechRecognizer/rpc")

if ret == false then
    print("roft-track-n-grasp-speech::main. Error: cannot connect with the speech recognizer.")
    os.exit()
end

---------------------------------------
-- functions Speech Recognition      --
---------------------------------------

objects = {"cracker", "mustard", "sugar"}

-- defining speech grammar in order to expand the speech recognition
grammar="Lets play with #Object"


function SM_RGM_Expand(port, vocab, word)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("RGM")
    wb:addString("vocabulory")
    wb:addString("add")
    wb:addString(vocab)
    wb:addString(word)
    port:write(wb)
    return "OK" --reply:get(1):asString()
end


function SM_Reco_Grammar(port, gram)
    local wb = yarp.Bottle()
    local reply = yarp.Bottle()
    wb:clear()
    wb:addString("recog")
    wb:addString("grammarSimple")
    wb:addString(gram)
    port:write(wb,reply)
    return reply
end


function send_output(port, cmd)
   --local wb = port:prepare()
   local wb = yarp.Bottle()
   wb:clear()
   wb=cmd
   port:write(wb)
end

print("roft-track-n-grasp-speech::main. Info: expanding the vocabulary.")
ret = true
for key, word in pairs(objects) do
    ret = ret and (SM_RGM_Expand(port_speech_recog, "#Object", word) == "OK")
end
if ret == false then
    print("roft-track-n-grasp-speech::main. Error: cannot expand the vocabulary.")
end

print("roft-track-n-grasp-speech::main. Info: ready to receive commands.")


---------------------------------------
-- Main loop                         --
---------------------------------------

while state ~= "quit" and not interrupting do

    local result = SM_Reco_Grammar(port_speech_recog, grammar)

    local speechcmd = result:get(1):asString()

    if speechcmd == "Lets" then
       local object_name = result:get(7):asString()

       local output = yarp.Bottle()
       output:addString("select_object")

       if object_name == "cracker" then
          output:addString("o003")
       elseif object_name == "mustard" then
          output:addString("o006")
       elseif object_name == "sugar" then
          output:addString("o004")
       end

       send_output(port_output, output)
    end

    yarp.delay(1.0)
end

port_speech_recog:close()
port_output:close()
yarp.Network_fini()