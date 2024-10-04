# frozen_string_literal: true

using_task_library "thrusters_blue_robotics_t500"

describe OroGen.thrusters_blue_robotics_t500.Task do
    run_live

    attr_reader :task
    before do
        @task = syskit_deploy(OroGen.thrusters_blue_robotics_t500.Task
                                    .deployed_as("t500_driver"))

        command_table_path = File.join(__dir__,
                                       "data/blue_robotics_t500-command_24V.csv")

        @task.properties.command_to_pwm_table_file_path = command_table_path
        @task.properties.no_actuation_pwm_command = 42
    end

    it "raises if cmd_in has different mode than configured" do
        task.properties.helices_alignment = [helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE)]
        cmd_in = effort_command({ a: 1, b: 2 })
        cmd_in.elements.push(Types.base.JointState.Raw(3))
        cmd_in.names.push("c")

        syskit_configure_and_start(task)
        expect_execution do
            syskit_write task.cmd_in_port, cmd_in
        end.to do
            emit task.invalid_command_mode_event
            emit task.exception_event
        end
    end

    it "raises if cmd_in has different size than helices alignment configured" do
        task.properties.helices_alignment = [helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE)]
        cmd_in = effort_command({ a: 1, b: 2 })
        cmd_in.elements.push(Types.base.JointState.Raw(3))
        cmd_in.names.push("c")

        syskit_configure_and_start(task)
        expect_execution do
            syskit_write task.cmd_in_port, cmd_in
        end.to do
            emit task.invalid_command_size_event
            emit task.exception_event
        end
    end

    it "saturates backwards" do
        task.properties.helices_alignment = [helice_alignment(:COUNTERCLOCKWISE)]
        syskit_configure_and_start(task)
        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write task.cmd_in_port, effort_command({ a: -10.32 })
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 1
        assert_equal 1100, pwm_out.duty_cycles[0]
    end

    it "saturates forward" do
        task.properties.helices_alignment = [helice_alignment(:COUNTERCLOCKWISE)]
        syskit_configure_and_start(task)
        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write task.cmd_in_port, effort_command({ a: 16.45 })
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 1
        assert_equal 1900, pwm_out.duty_cycles[0]
    end

    it "saturates backwards inverted helice" do
        task.properties.helices_alignment = [helice_alignment(:CLOCKWISE)]
        syskit_configure_and_start(task)
        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write task.cmd_in_port, effort_command({ a: -10.45 })
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 1
        assert_equal 1900, pwm_out.duty_cycles[0]
    end

    it "saturates forward inverted helice" do
        task.properties.helices_alignment = [helice_alignment(:CLOCKWISE)]
        syskit_configure_and_start(task)
        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write task.cmd_in_port, effort_command({ a: 17.32 })
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 1
        assert_equal 1100, pwm_out.duty_cycles[0]
    end

    it "sends null pwm command when input command is zero" do
        task.properties.helices_alignment = [helice_alignment(:COUNTERCLOCKWISE)]
        syskit_configure_and_start(task)
        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write task.cmd_in_port, effort_command({ a: 0 })
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 1
        assert_equal pwm_out.duty_cycles[0], task.properties.no_actuation_pwm_command
    end

    it "interpolates effort commands" do
        task.properties.helices_alignment = [helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE)]
        syskit_configure_and_start(task)

        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write(task.cmd_in_port,
                         effort_command({ a: 2.21, b: 6.21,
                                          c: 0.69, d: -2.86, e: -6.52 }))
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 5

        expected = [1589, 1695, 1539, 1353, 1229]

        expected.zip(pwm_out.duty_cycles).each do |true_value, actual|
            assert_equal true_value, actual
        end
    end

    it "interpolates effort commands with inverted helice" do
        task.properties.helices_alignment = [helice_alignment(:CLOCKWISE),
                                             helice_alignment(:CLOCKWISE),
                                             helice_alignment(:CLOCKWISE),
                                             helice_alignment(:CLOCKWISE),
                                             helice_alignment(:CLOCKWISE)]
        syskit_configure_and_start(task)

        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write(task.cmd_in_port,
                         effort_command({ a: 2.21, b: 6.21,
                                          c: 0.69, d: -2.86, e: -6.52 }))
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 5

        expected = [1411, 1305, 1461, 1647, 1771]

        expected.zip(pwm_out.duty_cycles).each do |true_value, actual|
            assert_equal true_value, actual
        end
    end

    it "allows both commands to go to maximum duty cycle despite the helice alignment" do
        task.properties.helices_alignment = [helice_alignment(:CLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE),
                                             helice_alignment(:CLOCKWISE),
                                             helice_alignment(:COUNTERCLOCKWISE)]
        syskit_configure_and_start(task)

        t0 = Time.now
        pwm_out = expect_execution do
            syskit_write(task.cmd_in_port,
                         effort_command({ a: 16.44, b: 16.44,
                                          c: -10.31, d: -10.31  }))
        end.to do
            have_one_new_sample task.cmd_out_port
        end

        assert pwm_out.timestamp > t0
        assert_equal pwm_out.duty_cycles.size, 4

        expected = [1100, 1900, 1900, 1100]

        expected.zip(pwm_out.duty_cycles).each do |true_value, actual|
            assert_equal true_value, actual
        end
    end

    def effort_command(values)
        Types.base.samples.Joints.new(
            time: Time.now,
            names: values.map { |k, _| k.to_s },
            elements: values.map { |_, x| Types.base.JointState.Effort(x) }
        )
    end

    def helice_alignment(alignment)
        Types.thrusters_blue_robotics_t500.HeliceAlignment.new(alignment)
    end
end
