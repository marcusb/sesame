import matter
import sesame

class Matter_Door_Plugin : matter.Plugin_Shutter
    def init(node_id, endpoint_id)
        super(self).init(node_id, endpoint_id)
        self.shadow_shutter_inverted = 1
        tasmota.log("Matter_Door_Plugin initialized", tasmota.LOG_INFO)
    end

    def update_shadow()
        # Get door state from Sesame
        # Expected return: {"Position": 0 or 100, "Direction": 0, 1, or -1}
        var state = sesame.get_door_state()
        if state != nil
            var pos = state.find("Position")
            if pos != nil
                if pos != self.shadow_shutter_pos
                    self.attribute_updated(0x0102, 0x000E)   # CurrentPositionLiftPercent100ths
                end
                self.shadow_shutter_pos = pos
            end
            var dir = state.find("Direction")
            if dir != nil
                if dir != self.shadow_shutter_direction
                    self.attribute_updated(0x0102, 0x000A)   # OperationalStatus
                end
                self.shadow_shutter_direction = dir
            end
        end
        super(self).update_shadow()
    end

    def invoke_request(session, val, ctx)
        var cluster = ctx.cluster
        var command = ctx.command

        if cluster == 0x0102
            if   command == 0x0000            # UpOrOpen
                sesame.door_cmd(1)
                return true
            elif command == 0x0001            # DownOrClose
                sesame.door_cmd(2)
                return true
            elif command == 0x0002            # StopMotion
                sesame.door_cmd(0)
                return true
            end
        end
        return super(self).invoke_request(session, val, ctx)
    end
end

return Matter_Door_Plugin
