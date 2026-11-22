"""rqt entry point."""

from rqt_gui_py.plugin import Plugin

from .flow_widget import ExecutionFlowWidget


class ExecutionFlowPlugin(Plugin):

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('ExecutionFlow')
        self._widget = ExecutionFlowWidget(context.node)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                f'{self._widget.windowTitle()} ({context.serial_number()})')
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(instance_settings)
